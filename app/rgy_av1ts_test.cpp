// AV1 in MPEG-2 TS 関連のテスト
// "Carriage of AV1 in MPEG-2 TS" (AOM Working Group Draft) に基づく実装の検証用
#include "rgy_bitstream.h"
#include "rgy_tsdemux.h"
#include <cstdio>
#include <cstring>
#include <vector>

static int g_fail = 0;
#define CHECK(cond, msg) do { if (!(cond)) { std::fprintf(stderr, "FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); g_fail++; } } while (0)

// 指定データ中に start code 形式で禁止される 3byte 列 (00 00 00 / 00 00 01 / 00 00 02) が
// tsOBU ペイロード内 (start code を除く) に現れないことを確認する
static bool check_tsobu_payload(const std::vector<uint8_t>& tsobu) {
    static const uint8_t SC[3] = { 0x00, 0x00, 0x01 };
    size_t pos = 0;
    int obuCount = 0;
    while (pos + 3 <= tsobu.size()) {
        CHECK(pos + 3 <= tsobu.size() && std::memcmp(tsobu.data() + pos, SC, 3) == 0, "tsOBU starts with 0x000001");
        if (std::memcmp(tsobu.data() + pos, SC, 3) != 0) return false;
        pos += 3;
        obuCount++;
        // 次の start code までがこの OBU のペイロード
        size_t next = pos;
        while (next + 3 <= tsobu.size() && std::memcmp(tsobu.data() + next, SC, 3) != 0) next++;
        for (size_t i = pos; i + 3 <= next; i++) {
            if (tsobu[i] == 0 && tsobu[i + 1] == 0 && tsobu[i + 2] <= 0x02) {
                std::fprintf(stderr, "forbidden sequence 00 00 %02x at payload offset %zu\n", tsobu[i + 2], i - pos);
                return false;
            }
            // 00 00 03 は 00..03 の後続のみ許可
            if (tsobu[i] == 0 && tsobu[i + 1] == 0 && tsobu[i + 2] == 0x03 && i + 3 < next && tsobu[i + 3] > 0x03) {
                std::fprintf(stderr, "invalid emulation prevention 00 00 03 %02x at payload offset %zu\n", tsobu[i + 3], i - pos);
                return false;
            }
        }
        pos = next;
    }
    return obuCount > 0;
}

// low overhead 形式の OBU 群を作るヘルパ (obu_header + uleb size + payload)
static std::vector<uint8_t> make_low_obu(uint8_t type, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> obu;
    obu.push_back(gen_obu_header(type)); // has_size_flag=1
    const auto sz = get_av1_uleb_size_data(payload.size());
    obu.insert(obu.end(), sz.begin(), sz.end());
    obu.insert(obu.end(), payload.begin(), payload.end());
    return obu;
}

static bool test_parse_obu_low_overhead() {
    std::vector<uint8_t> stream;
    auto td = make_low_obu(OBU_TEMPORAL_DELIMITER, {});
    auto sh = make_low_obu(OBU_SEQUENCE_HEADER, { 0x0a, 0x0b, 0x00, 0x00, 0x05, 0x80 }); //わざと 00 00 を含む
    auto fh = make_low_obu(OBU_FRAME_HEADER, { 0x12 });
    auto frame = make_low_obu(OBU_FRAME, { 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04 }); //わざと start code 似を含む
    stream.insert(stream.end(), td.begin(), td.end());
    stream.insert(stream.end(), sh.begin(), sh.end());
    stream.insert(stream.end(), fh.begin(), fh.end());
    stream.insert(stream.end(), frame.begin(), frame.end());

    const auto obus = parse_obu_av1(stream.data(), stream.size());
    CHECK(obus.size() == 4, "low overhead: 4 OBUs");
    if (obus.size() == 4) {
        CHECK(obus[0].type == OBU_TEMPORAL_DELIMITER, "obu0 type TD");
        CHECK(obus[1].type == OBU_SEQUENCE_HEADER, "obu1 type SH");
        CHECK(obus[2].type == OBU_FRAME_HEADER, "obu2 type FH");
        CHECK(obus[3].type == OBU_FRAME, "obu3 type FRAME");
        CHECK(obus[3].size == frame.size(), "obu3 size");
        CHECK(!obus[3].escaped, "low overhead is not escaped");
    }

    // tsOBU 変換: start code が付き、emulation prevention されていること
    const auto tsobu = av1_to_tsobu(stream.data(), stream.size());
    CHECK(check_tsobu_payload(tsobu), "tsOBU payload has no forbidden sequences");
    // 変換結果を再度 parse すると同じ OBU 数・type になること (roundtrip)
    const auto obus2 = parse_obu_av1(tsobu.data(), tsobu.size());
    CHECK(obus2.size() == 4, "tsOBU roundtrip: 4 OBUs");
    if (obus2.size() == 4) {
        CHECK(obus2[1].type == OBU_SEQUENCE_HEADER, "roundtrip obu1 type SH");
        CHECK(obus2[3].type == OBU_FRAME, "roundtrip obu3 type FRAME");
        // emulation prevention が除去されて元の payload が復元できること
        std::vector<uint8_t> raw = unnal(obus2[3].ptr, obus2[3].size);
        CHECK(raw == frame, "roundtrip obu3 payload restored");
    }
    return g_fail == 0;
}

static bool test_parse_obu_startcode() {
    // tsOBU 形式 (start code + emulation prevention 済み) の入力
    std::vector<uint8_t> payload = { 0x0a, 0x00, 0x00, 0x00, 0x01, 0xff }; // 00 00 00 / 00 00 01 を含む
    std::vector<uint8_t> escaped = payload;
    to_nal(escaped);
    std::vector<uint8_t> stream = { 0x00, 0x00, 0x01 };
    stream.push_back(gen_obu_header(OBU_SEQUENCE_HEADER));
    const auto sz = get_av1_uleb_size_data(escaped.size());
    stream.insert(stream.end(), sz.begin(), sz.end());
    stream.insert(stream.end(), escaped.begin(), escaped.end());
    stream.insert(stream.end(), { 0x00, 0x00, 0x01 });
    stream.push_back(gen_obu_header(OBU_FRAME));
    stream.insert(stream.end(), { 0x00, 0x00, 0x02, 0x00, 0x00, 0x03, 0x99 }); // emulation prevention 込みデータ

    const auto obus = parse_obu_av1(stream.data(), stream.size());
    CHECK(obus.size() == 2, "startcode: 2 OBUs");
    if (obus.size() == 2) {
        CHECK(obus[0].type == OBU_SEQUENCE_HEADER && obus[0].escaped, "obu0 SH escaped");
        CHECK(obus[1].type == OBU_FRAME && obus[1].escaped, "obu1 FRAME escaped");
        std::vector<uint8_t> raw0 = unnal(obus[0].ptr, obus[0].size);
        std::vector<uint8_t> expect0;
        expect0.push_back(gen_obu_header(OBU_SEQUENCE_HEADER));
        expect0.insert(expect0.end(), sz.begin(), sz.end());
        expect0.insert(expect0.end(), payload.begin(), payload.end());
        CHECK(raw0 == expect0, "startcode obu0 unescaped correctly");
    }
    // av1_to_tsobu に掛けても禁止列が出ないこと
    const auto tsobu = av1_to_tsobu(stream.data(), stream.size());
    CHECK(check_tsobu_payload(tsobu), "startcode->tsOBU payload has no forbidden sequences");
    return g_fail == 0;
}

// 手作りの sequence_header_obu を作る (ビット列を手書き)
// seq_profile=0, still=0, reduced=0, timing_info_present=0, decoder_model_present=0,
// initial_display_delay_present=0, operating_points_cnt_minus1=0, op_idc=0, seq_level_idx=8 (5bit), 
// frame_width_bits_minus1=15, frame_height_bits_minus1=15, max_w=1919(16bit), max_h=1079(16bit),
// frame_id_present=0, use_128=0, filter_intra=0, intra_edge=0,
// interintra=0, masked=0, warped=0, dual=0, order_hint=0, choose_screen=1 (force=2), 
// superres=0, cdef=0, restoration=0,
// high_bitdepth=1 (profile0なので twelve_bit読まない), mono=0, color_desc_present=1, cp=1, tc=1, mc=1,
// color_range=1, profile0なので sub_x=1,sub_y=1, csp f(2)=0, separate_uv_delta_q=0, trailing 0x80
static std::vector<uint8_t> make_seq_header_obu() {
    std::vector<uint8_t> bits;
    auto f = [&](uint32_t v, int n) { for (int i = n - 1; i >= 0; i--) bits.push_back((v >> i) & 1); };
    f(0, 3);   // seq_profile
    f(0, 1);   // still_picture
    f(0, 1);   // reduced_still_picture_header
    f(0, 1);   // timing_info_present_flag
    // decoder_model_info_present_flag は timing_info_present_flag=1 のときのみ存在するため書かない
    f(0, 1);   // initial_display_delay_present_flag
    f(0, 5);   // operating_points_cnt_minus1
    f(0, 12);  // operating_point_idc[0]
    f(8, 5);   // seq_level_idx[0] = 8
    f(0, 1);   // seq_tier[0] (seq_level_idx > 7 なので tier ビットが続く、AV1 spec 5.5.2)
    f(15, 4);  // frame_width_bits_minus1
    f(15, 4);  // frame_height_bits_minus1
    f(1919, 16); // max_frame_width_minus1
    f(1079, 16); // max_frame_height_minus1
    f(0, 1);   // frame_id_numbers_present_flag
    f(0, 1);   // use_128x128_superblock
    f(0, 1);   // enable_filter_intra
    f(0, 1);   // enable_intra_edge_filter
    f(0, 1);   // enable_interintra_compound
    f(0, 1);   // enable_masked_compound
    f(0, 1);   // enable_warped_motion
    f(0, 1);   // enable_dual_filter
    f(0, 1);   // enable_order_hint
    f(1, 1);   // seq_choose_screen_content_tools -> force = 2
    // seq_force_screen_content_tools > 0 -> seq_choose_integer_mv
    f(1, 1);   // seq_choose_integer_mv -> force = SELECT (2), 追加読みなし
    f(0, 1);   // enable_superres
    f(0, 1);   // enable_cdef
    f(0, 1);   // enable_restoration
    f(1, 1);   // high_bitdepth (profile 0 -> twelve_bit 読まない)
    f(0, 1);   // mono_chrome
    f(1, 1);   // color_description_present_flag
    f(1, 8);   // color_primaries = BT.709
    f(1, 8);   // transfer_characteristics = BT.709
    f(1, 8);   // matrix_coefficients = BT.709
    f(1, 1);   // color_range
    // profile 0 -> subsampling_x=y=1 (読まない)
    // subsampling_x && subsampling_y -> chroma_sample_position f(2)
    f(0, 2);   // chroma_sample_position
    f(0, 1);   // separate_uv_delta_q
    // trailing bits
    bits.push_back(1);
    while (bits.size() % 8 != 0) bits.push_back(0);
    std::vector<uint8_t> payload;
    for (size_t i = 0; i < bits.size(); i += 8) {
        uint8_t b = 0;
        for (int j = 0; j < 8; j++) b = (b << 1) | bits[i + j];
        payload.push_back(b);
    }
    return make_low_obu(OBU_SEQUENCE_HEADER, payload);
}

static bool test_seq_header_parse() {
    const auto sh = make_seq_header_obu();
    RGYAV1SeqHeaderInfo info;
    CHECK(parse_av1_sequence_header_obu(sh.data(), sh.size(), info), "parse seq header obu");
    CHECK(info.valid, "seq header valid");
    CHECK(info.seq_profile == 0, "seq_profile == 0");
    CHECK(info.seq_level_idx_0 == 8, "seq_level_idx_0 == 8");
    CHECK(info.seq_tier_0 == 0, "seq_tier_0 == 0");
    CHECK(info.high_bitdepth == 1, "high_bitdepth == 1");
    CHECK(info.twelve_bit == 0, "twelve_bit == 0");
    CHECK(info.monochrome == 0, "monochrome == 0");
    CHECK(info.chroma_subsampling_x == 1 && info.chroma_subsampling_y == 1, "subsampling 1,1");
    CHECK(info.chroma_sample_position == 0, "csp == 0");
    CHECK(info.color_primaries == 1 && info.transfer_characteristics == 1 && info.matrix_coefficients == 1, "color desc 1,1,1");

    // ストリームから探す (TD + SH)
    std::vector<uint8_t> stream;
    auto td = make_low_obu(OBU_TEMPORAL_DELIMITER, {});
    stream.insert(stream.end(), td.begin(), td.end());
    stream.insert(stream.end(), sh.begin(), sh.end());
    RGYAV1SeqHeaderInfo info2;
    CHECK(find_av1_sequence_header(stream.data(), stream.size(), info2), "find seq header in stream");
    CHECK(info2.valid && info2.seq_level_idx_0 == 8, "found seq header level 8");

    // AV1 video descriptor の確認 (tag 0x80, len 4, marker/version, profile/level, ...)
    const auto desc = gen_av1_video_descriptor(info);
    CHECK(desc.size() == 6, "descriptor size 6");
    if (desc.size() == 6) {
        CHECK(desc[0] == 0x80 && desc[1] == 0x04, "descriptor tag/len");
        CHECK(desc[2] == 0x81, "marker=1 version=1");
        CHECK(desc[3] == (uint8_t)((0 << 5) | 8), "profile/level byte");
        CHECK(desc[4] == (uint8_t)(0x40 | (1 << 3) | (1 << 2) | 0), "tier/bitdepth/subsampling byte");
        CHECK((desc[5] >> 5) == 0, "hdr_wcg_idc == 0 (SDR BT.709)");
    }

    // av1C レコードからの読み取り
    // marker=1 version=1 | profile=2 level=19 | tier=1 high=1 twelve=1 mono=0 subx=1 suby=0 csp=2 | reserved
    const uint8_t av1c[4] = { 0x81, (uint8_t)((2 << 5) | 19), (uint8_t)(0x80 | 0x40 | 0x20 | 0x08 | (2)), 0x00 };
    RGYAV1SeqHeaderInfo info3;
    CHECK(parse_av1_codec_config_record(av1c, sizeof(av1c), info3), "parse av1C");
    CHECK(info3.valid && info3.seq_profile == 2 && info3.seq_level_idx_0 == 19, "av1C profile/level");
    CHECK(info3.seq_tier_0 == 1 && info3.high_bitdepth == 1 && info3.twelve_bit == 1, "av1C tier/bitdepth");
    CHECK(info3.monochrome == 0 && info3.chroma_subsampling_x == 1 && info3.chroma_subsampling_y == 0 && info3.chroma_sample_position == 2, "av1C chroma");
    const auto desc3 = gen_av1_video_descriptor(info3);
    CHECK(desc3.size() == 6 && desc3[3] == (uint8_t)((2 << 5) | 19), "av1C descriptor byte1");

    // 実際のSVT-AV1出力 (4K放送 TS から得た sequence header OBU, level_idx=18 > 7 で tier ビットあり) の回帰テスト
    {
        const uint8_t svtShPayload[] = { 0x02, 0x00, 0x00, 0x4a, 0x95, 0x5d, 0xfe, 0x1b, 0x8d, 0x5f, 0x32, 0x02, 0x02, 0x02, 0x08 };
        auto svtSh = make_low_obu(OBU_SEQUENCE_HEADER, std::vector<uint8_t>(svtShPayload, svtShPayload + sizeof(svtShPayload)));
        RGYAV1SeqHeaderInfo infoSvt;
        CHECK(parse_av1_sequence_header_obu(svtSh.data(), svtSh.size(), infoSvt), "parse SVT seq header");
        CHECK(infoSvt.valid, "SVT seq header valid");
        CHECK(infoSvt.seq_profile == 0 && infoSvt.seq_level_idx_0 == 9 && infoSvt.seq_tier_0 == 0, "SVT profile/level/tier");
        CHECK(infoSvt.high_bitdepth == 0 && infoSvt.monochrome == 0, "SVT bitdepth/mono");
    }
    return g_fail == 0;
}

// PMT に AV1 (stream_type 0x06 + registration 'AV01') の映像エントリがある場合に
// 映像PIDとして認識されるかを確認する
static std::vector<uint8_t> makeSectionPacket(uint16_t pid, bool pusi, uint8_t cc, const std::vector<uint8_t>& section) {
    std::vector<uint8_t> pkt(188, 0xff);
    pkt[0] = 0x47;
    pkt[1] = (pusi ? 0x40 : 0x00) | (uint8_t)(pid >> 8);
    pkt[2] = (uint8_t)(pid & 0xff);
    pkt[3] = 0x10 | (cc & 0x0f);
    size_t pos = 4;
    if (pusi) pkt[pos++] = 0x00; // pointer_field
    std::memcpy(pkt.data() + pos, section.data(), section.size());
    return pkt;
}

static std::vector<uint8_t> makePAT() {
    // PAT: tsid=1, program 1 -> PMT pid 0x1000
    std::vector<uint8_t> s;
    s.push_back(0x00); // table_id
    s.push_back(0xb0); s.push_back(13); // section_length = 13
    s.push_back(0x00); s.push_back(0x01); // tsid
    s.push_back(0xc1); // version 0, current_next
    s.push_back(0x00); s.push_back(0x00); // section_number, last_section_number
    s.push_back(0x00); s.push_back(0x01); // program_number 1
    s.push_back(0xe0 | (0x10)); s.push_back(0x00); // PMT pid 0x1000
    // CRC32 (calc_crc32 は rgy_tsutil.h で定義)
    const uint32_t crc = calc_crc32(s.data(), (int)s.size());
    s.push_back((uint8_t)(crc >> 24)); s.push_back((uint8_t)(crc >> 16)); s.push_back((uint8_t)(crc >> 8)); s.push_back((uint8_t)crc);
    return s;
}

static std::vector<uint8_t> makePMTWithAV1() {
    // PMT: program 1, PCR pid = video pid 0x0100, ES: stream_type 0x06 pid 0x0100,
    // descriptors: registration 'AV01' (0x05) + AV1 video descriptor (0x80) + stream identifier (0x52)
    std::vector<uint8_t> s;
    s.push_back(0x02); // table_id PMT
    const size_t lenPos = s.size();
    s.push_back(0xb0); s.push_back(0x00); // section_length (後で設定)
    s.push_back(0x00); s.push_back(0x01); // program_number
    s.push_back(0xc1); // version 0
    s.push_back(0x00); s.push_back(0x00);
    s.push_back(0xe0 | (0x01)); s.push_back(0x00); // PCR pid 0x0100
    s.push_back(0xf0); s.push_back(0x00); // program_info_length 0
    // ES entry
    s.push_back(0x06); // stream_type 0x06 (PES private data)
    s.push_back(0xe0 | 0x01); s.push_back(0x00); // pid 0x0100
    const size_t esLenPos = s.size();
    s.push_back(0xf0); s.push_back(0x00); // ES_info_length (後で設定)
    // registration descriptor 'AV01' (先頭に置くことが必須)
    s.push_back(0x05); s.push_back(0x04); s.push_back('A'); s.push_back('V'); s.push_back('0'); s.push_back('1');
    // AV1 video descriptor
    const uint8_t av1desc[6] = { 0x80, 0x04, 0x81, 0x08, 0x4c, 0x00 };
    s.insert(s.end(), av1desc, av1desc + 6);
    // stream identifier descriptor
    s.push_back(0x52); s.push_back(0x01); s.push_back(0x00);
    const int esLen = (int)(s.size() - esLenPos - 2);
    s[esLenPos + 1] = (uint8_t)esLen;
    const int sectionLen = (int)(s.size() - 3) + 4; // CRC 込み
    s[lenPos] = (uint8_t)(0xb0 | ((sectionLen >> 8) & 0x0f));
    s[lenPos + 1] = (uint8_t)(sectionLen & 0xff);
    const uint32_t crc = calc_crc32(s.data(), (int)s.size());
    s.push_back((uint8_t)(crc >> 24)); s.push_back((uint8_t)(crc >> 16)); s.push_back((uint8_t)(crc >> 8)); s.push_back((uint8_t)crc);
    return s;
}

static bool test_pmt_av1_detect() {
    RGYTSDemuxer demuxer;
    demuxer.init(nullptr, 0);
    auto patPkt = makeSectionPacket(0x0000, true, 0, makePAT());
    RGYTSPacket patTs;
    patTs.packet = patPkt;
    // ヘッダを自分で埋める (parse は header を参照する)
    patTs.header.Sync = 0x47;
    patTs.header.PID = 0x0000;
    patTs.header.PayloadStartFlag = 1;
    patTs.header.Counter = 0;
    patTs.header.payloadSize = 188 - 4; // ヘッダ4byteのみ、payload は pointer_field から
    patTs.header.adaptation = 1;
    auto [err1, res1] = demuxer.parse(&patTs);
    CHECK(err1 == RGY_ERR_NONE && res1.type == RGYTSPacketType::PAT, "PAT parsed");

    auto pmtPkt = makeSectionPacket(0x1000, true, 0, makePMTWithAV1());
    RGYTSPacket pmtTs;
    pmtTs.packet = pmtPkt;
    pmtTs.header.Sync = 0x47;
    pmtTs.header.PID = 0x1000;
    pmtTs.header.PayloadStartFlag = 1;
    pmtTs.header.Counter = 0;
    pmtTs.header.payloadSize = 188 - 4;
    pmtTs.header.adaptation = 1;
    auto [err2, res2] = demuxer.parse(&pmtTs);
    CHECK(err2 == RGY_ERR_NONE && res2.type == RGYTSPacketType::PMT, "PMT parsed");
    const auto service = demuxer.service();
    CHECK(service != nullptr, "service exists");
    if (service) {
        CHECK(service->vid.stream.pid == 0x0100, "AV1 pid detected as video pid");
        CHECK(service->vid.stream.type == RGYTSStreamType::AV1_VIDEO, "AV1 stream type detected");
        CHECK(service->cap.stream.pid == 0, "not treated as caption");
    }
    return g_fail == 0;
}

static bool test_pmt_av1_registration_helper() {
    const uint8_t desc[] = { 0x05, 0x04, 'A', 'V', '0', '1', 0x52, 0x01, 0x00 };
    CHECK(rgyTSESInfoHasAV1Registration(desc, sizeof(desc)), "detect AV01 registration");
    const uint8_t desc2[] = { 0x52, 0x01, 0x00, 0x05, 0x04, 'A', 'V', '0', '1' };
    CHECK(rgyTSESInfoHasAV1Registration(desc2, sizeof(desc2)), "detect AV01 registration (not first)");
    const uint8_t desc3[] = { 0x05, 0x04, 'H', 'E', 'V', 'C' };
    CHECK(!rgyTSESInfoHasAV1Registration(desc3, sizeof(desc3)), "HEVC registration is not AV1");
    const uint8_t desc4[] = { 0x52, 0x01, 0x30 };
    CHECK(!rgyTSESInfoHasAV1Registration(desc4, sizeof(desc4)), "no registration");
    return g_fail == 0;
}

int main() {
    test_parse_obu_low_overhead();
    test_parse_obu_startcode();
    test_seq_header_parse();
    test_pmt_av1_registration_helper();
    test_pmt_av1_detect();
    if (g_fail == 0) {
        std::printf("all AV1 TS tests passed.\n");
        return 0;
    }
    std::fprintf(stderr, "%d test(s) failed.\n", g_fail);
    return 1;
}
