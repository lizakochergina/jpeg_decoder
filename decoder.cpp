#include "decoder.h"
#include <stdexcept>
#include <iostream>
#include <vector>

std::map<std::string, std::shared_ptr<Marker>> MarkersTable::markers = {
    {"sof0", std::make_shared<SOF0>()}, {"dht", std::make_shared<DHT>()},
    {"dqt", std::make_shared<DQT>()},   {"appn", std::make_shared<APPn>()},
    {"com", std::make_shared<COM>()},   {"sos", std::make_shared<SOS>()},
    {"eoi", std::make_shared<EOI>()}};

Image Decode(std::istream& input) {
    Reader myreader(&input);
    ImageData img;
    myreader.Process(img);

    Quantization(img);
    IDCTTransformation(img);

    ImageCreator img_creator(&img);
    return img_creator.Create();
}

void Reader::Process(ImageData& img) {
    MarkersTable mt;

    NextMarker();
    if (eof_) {
        throw std::invalid_argument("empty input");
    }
    if (next_marker_ != "soi") {
        throw std::invalid_argument("soi is missing");
    }

    img.dht_tables.resize(2);
    NextMarker();
    while (!eof_) {
        if (mt.markers.find(next_marker_) == mt.markers.end()) {
            throw std::runtime_error("unknown marker\n");
        }
        mt.markers[next_marker_]->ProcessMarker(*this, img);
        NextMarker();
    }

    if (next_marker_ != "eoi") {
        throw std::invalid_argument("eoi is missing");
    }
}

void Quantization(ImageData& img) {
    int k = 1;
    for (auto& [y_matrix, cb_matrix, cr_matrix] : img.mcu) {
        uint id = 0;
        uint quant_matrix_id = img.channels[id].quant_table_id;
        for (int i = 0; i != y_matrix.size(); ++i) {
            for (int j = 0; j != 64; ++j) {
                y_matrix[i][j] *= img.dqt_tables[quant_matrix_id][j];
            }
        }

        if (img.channels.size() > 1) {
            ++id;
            quant_matrix_id = img.channels[id].quant_table_id;
            for (int j = 0; j != 64; ++j) {
                cb_matrix[j] *= img.dqt_tables[quant_matrix_id][j];
            }

            ++id;
            quant_matrix_id = img.channels[id].quant_table_id;
            for (int j = 0; j != 64; ++j) {
                cr_matrix[j] *= img.dqt_tables[quant_matrix_id][j];
            }
        }
        ++k;
    }
}

void IDCTTransformation(ImageData& img) {
    int k = 1;

    std::vector<double> input;
    input.resize(64);
    std::vector<double> output;
    output.resize(64);
    DctCalculator dst(8, &input, &output);

    for (auto& [y_matrix, cb_matrix, cr_matrix] : img.mcu) {
        for (int i = 0; i != y_matrix.size(); ++i) {
            for (int j = 0; j != y_matrix[i].size(); ++j) {
                input[j] = y_matrix[i][j];
            }
            dst.Inverse();
            for (int j = 0; j != y_matrix[i].size(); ++j) {
                int tmp = static_cast<int>(round(output[j]));
                y_matrix[i][j] = std::min(std::max(0, tmp + 128), 255);
            }
        }

        if (img.channels.size() > 1) {
            for (int i = 0; i != cb_matrix.size(); ++i) {
                input[i] = cb_matrix[i];
            }
            dst.Inverse();
            for (int i = 0; i != cb_matrix.size(); ++i) {
                int tmp = static_cast<int>(round(output[i]));
                cb_matrix[i] = std::min(std::max(0, tmp + 128), 255);
            }

            for (int i = 0; i != cr_matrix.size(); ++i) {
                input[i] = cr_matrix[i];
            }
            dst.Inverse();
            for (int i = 0; i != cr_matrix.size(); ++i) {
                int tmp = static_cast<int>(round(output[i]));
                cr_matrix[i] = std::min(std::max(0, tmp + 128), 255);
            }
        }
        ++k;
    }
}

void Reader::NextMarker() {
    int next = in_->peek();
    if (next == EOF) {
        eof_ = true;
        return;
    }
    if (next == 0xFF || last_was_ff_) {
        if (next == 0xFF) {
            in_->get();
            next = in_->peek();
            if (next == EOF) {
                throw std::invalid_argument("incorrect marker");
            }
        }
        next = in_->get();
        eof_ = false;
        if (next == 0xD8) {
            next_marker_ = "soi";
            return;
        } else if (next == 0xC0) {
            next_marker_ = "sof0";
            return;
        } else if (next == 0xC4) {
            next_marker_ = "dht";
            return;
        } else if (next == 0xDB) {
            next_marker_ = "dqt";
            return;
        } else if (0xE0 <= next && next <= 0xEF) {
            next_marker_ = "appn";
            return;
        } else if (next == 0xFE) {
            next_marker_ = "com";
            return;
        } else if (next == 0xD9) {
            next_marker_ = "eoi";
            eof_ = true;
            return;
        } else if (next == 0xDA) {
            next_marker_ = "sos";
            return;
        } else {
            throw std::invalid_argument("incorrect marker");
        }
    }
    // else throw smth ??
}

unsigned int Reader::GetByte() {
    if (in_->peek() == EOF) {
        throw std::invalid_argument("unexpected end");
    }
    return in_->get();
}

uint Reader::GetBit() {
    if (bit_buff_.empty()) {
        ReadBits();
    }
    if (bit_buff_.empty()) {
        return 0;
    }
    auto bit = bit_buff_.back();
    bit_buff_.pop_back();
    return bit;
}

void Reader::ReadBits() {
    uint byte = GetByte();
    if (byte == 0xFF) {
        uint next = in_->peek();
        if (in_->peek() == 0xD9) {
            last_was_ff_ = true;
            return;
        } else if (in_->peek() != 0) {
            throw std::runtime_error("smth strange after 0xFF in sos section");
        }
        GetByte();
    }
    bit_buff_.clear();
    for (int i = 0; i != 8; ++i) {
        bit_buff_.push_back(byte & 1);
        byte = byte >> 1;
    }
}

bool Reader::IsEOF() {
    return in_->peek() == EOF;
}

bool Reader::NextIsMarker() {
    if (bit_buff_.empty()) {
        auto cur = in_->peek();
        if (cur != 0xFF) {
            return false;
        }
        cur = in_->get();
        auto next = in_->peek();
        in_->unget();
        if (next == 0) {
            return false;
        } else {
            return true;
        }
    }
    return false;
}

void SOF0::ProcessMarker(Reader& reader, ImageData& img) {
    if (img.sof0_check) {
        throw std::runtime_error("sof0 has already been");
    }
    img.sof0_check = true;

    unsigned int len = (reader.GetByte() << 8) + reader.GetByte() - 3;
    img.precision = reader.GetByte();

    uint h1 = reader.GetByte();
    uint h2 = reader.GetByte();
    uint h = (h1 << 8) + h2;

    uint w1 = reader.GetByte();
    uint w2 = reader.GetByte();
    uint w = (w1 << 8) + w2;

    if (h * w == 0) {
        throw std::runtime_error("zero size\n");
    }

    uint channels_cnt = reader.GetByte();
    if (channels_cnt != 1 && channels_cnt != 3) {
        throw std::runtime_error("invalid cnt of channels\n");
    }

    img.height = h;
    img.width = w;

    uint hmax = 0, vmax = 0;
    img.channels.resize(channels_cnt);
    for (int i = 0; i != channels_cnt; ++i) {
        uint id = reader.GetByte() - 1;  // есть 1 2 3 -> 0 1 2
        uint thinning = reader.GetByte();
        img.channels[id].h = thinning >> 4;
        img.channels[id].v = thinning & 0xf;
        img.channels[id].quant_table_id = reader.GetByte();
        hmax = std::max(hmax, img.channels[id].h);
        vmax = std::max(vmax, img.channels[id].v);
    }

    int i = 0;
    for (auto& [hor, ver, id1, id2, id3] : img.channels) {
        hor = hmax / hor;
        ver = vmax / ver;
    }
}

void DHT::ProcessMarker(Reader& reader, ImageData& img) {
    unsigned int len = (reader.GetByte() << 8) + reader.GetByte() - 2;
    while (len > 0) {
        len -= 17;
        unsigned int info_byte = reader.GetByte();
        auto coef_class = info_byte >> 4;
        auto table_id = (info_byte & 0xf);

        if (coef_class != 0 && coef_class != 1) {
            throw std::runtime_error("invalid class ac/dc\n");
        }

        std::vector<uint8_t> code_lenght(16, 0);
        std::vector<uint8_t> values;
        uint cnt = 0;
        for (size_t i = 0; i != 16; ++i) {
            code_lenght[i] = reader.GetByte();
            cnt += code_lenght[i];
        }
        len -= cnt;
        for (size_t i = 0; i != cnt; ++i) {
            values.push_back(reader.GetByte());
        }
        img.dht_tables[coef_class][table_id].Build(code_lenght, values);
    }
}

void DQT::ProcessMarker(Reader& reader, ImageData& img) {
    unsigned int len = (reader.GetByte() << 8) + reader.GetByte() - 2;
    while (len > 0) {
        --len;
        unsigned int info_byte = reader.GetByte();
        auto len_of_items = info_byte >> 4;
        auto table_id = info_byte & 0xf;
        img.dqt_tables[table_id].resize(64);
        int i = 0, j = 0, add = 1;
        for (size_t k = 0; k != 64; ++k) {
            auto cur = reader.GetByte();
            --len;
            if (len_of_items == 1) {
                cur = (cur << 8) + reader.GetByte();
                --len;
            }
            int id = (i << 3) + j;
            img.dqt_tables[table_id][(i << 3) + j] = cur;
            if (i == 0 && j == 0) {
                ++j;
            } else if (i == 7 && add == 1) {
                ++j;
                add = -1;
            } else if (j == 7 && add == -1) {
                ++i;
                add = 1;
            } else if (j == 0 && add == 1) {
                ++i;
                add = -1;
            } else if (i == 0 && add == -1) {
                ++j;
                add = 1;
            } else {
                j -= add;
                i += add;
            }
        }
    }
}

void APPn::ProcessMarker(Reader& reader, ImageData& img) {
    unsigned int len = (reader.GetByte() << 8) + reader.GetByte() - 2;
    while (len > 0) {
        reader.GetByte();
        --len;
    }
}

void COM::ProcessMarker(Reader& reader, ImageData& img) {
    unsigned int len = (reader.GetByte() << 8) + reader.GetByte() - 2;
    while (len > 0) {
        img.comment += reader.GetByte();
        --len;
    }
}

void SOS::ProcessMarker(Reader& reader, ImageData& img) {
    if (img.sos_check) {
        throw std::runtime_error("sof0 has already been");
    }
    img.sos_check = true;

    unsigned int len = (reader.GetByte() << 8) + reader.GetByte() - 3;
    uint channels_cnt = reader.GetByte();
    std::vector<uint> id_order;
    for (size_t i = 0; i != channels_cnt; ++i) {
        uint id = reader.GetByte() - 1;  // 1 2 3 -> 0 1 2
        if (id >= img.channels.size()) {
            throw std::runtime_error("wrong id of channel in sos\n");
        }
        id_order.push_back(id);
        uint id_tables = reader.GetByte();
        img.channels[id].dc_table_id = id_tables >> 4;
        img.channels[id].ac_table_id = id_tables & 0xf;
        len -= 2;
    }

    auto tmp1 = reader.GetByte();
    auto tmp2 = reader.GetByte();
    auto tmp3 = reader.GetByte();
    if (tmp1 != 0 || tmp2 != 0x3f || tmp3 != 0) {
        throw std::runtime_error("incorrect last 3 bytes of sos");
    }
    len -= 3;
    if (len != 0) {
        throw std::runtime_error("incorrect sos sec");
    }

    uint expected_mcu_cnt = GetMcuCnt(img);

    uint cnt_y = GetCntY(img);
    std::vector<int> last_dc(channels_cnt, 0);
    int k = 0;
    while (k < expected_mcu_cnt && !reader.IfLastWasFF() && !reader.IsEOF() &&
           !reader.NextIsMarker()) {
        img.mcu.push_back(MCU{});
        for (auto channel_id : id_order) {
            if (channel_id == 0) {
                for (uint i = 0; i != cnt_y; ++i) {
                    img.mcu.back().y_matrix.push_back(GetMatrix(reader, img, channel_id));
                }
                for (uint i = 0; i != cnt_y; ++i) {
                    img.mcu.back().y_matrix[i][0] += last_dc[channel_id];
                    last_dc[channel_id] = img.mcu.back().y_matrix[i][0];
                }
            } else if (channel_id == 1) {
                img.mcu.back().cb_matrix = GetMatrix(reader, img, channel_id);
                img.mcu.back().cb_matrix[0] += last_dc[channel_id];
                last_dc[channel_id] = img.mcu.back().cb_matrix[0];
            } else {
                img.mcu.back().cr_matrix = GetMatrix(reader, img, channel_id);
                img.mcu.back().cr_matrix[0] += last_dc[channel_id];
                last_dc[channel_id] = img.mcu.back().cr_matrix[0];
            }
        }
        ++k;
    }
    ++k;
}

std::vector<int> SOS::GetMatrix(Reader& reader, ImageData& img, uint channel_id) {
    uint dc = img.channels[channel_id].dc_table_id;
    uint ac = img.channels[channel_id].ac_table_id;
    std::vector<int> coefs;

    int val;
    while (!img.dht_tables[0][dc].Move(reader.GetBit(), val)) {
    }
    if (val == 0) {
        coefs.push_back(val);
    } else {
        FillCoefs(reader, img, val, coefs);
    }

    uint bit = reader.GetBit();
    while (!reader.IfLastWasFF() && coefs.size() < 64) {
        while (!img.dht_tables[1][ac].Move(bit, val)) {
            bit = reader.GetBit();
            if (reader.IfLastWasFF()) {
                throw std::invalid_argument("an unknown interrupt in sos section");
            }
        }
        // got a val in the huffman table

        if (val == 0) {  // left values are zeros
            break;
        }
        FillCoefs(reader, img, val, coefs);
        if (coefs.size() >= 64) {
            break;
        }
        bit = reader.GetBit();
    }
    if (coefs.size() > 64) {
        throw std::runtime_error("incorrect size of matrix in the mcu, >64");
    }

    // change to zig zar order
    return TransformToZigZag(coefs);
}

void SOS::FillCoefs(Reader& reader, ImageData& img, int val, std::vector<int>& coefs) {
    int cnt_zeros = val >> 4;
    int cnt_bits = val & 0xf;
    for (int i = 0; i != cnt_zeros; ++i) {
        coefs.push_back(0);
    }
    int real_val = 0;
    bool inverse = false;
    for (int i = 0; i != cnt_bits; ++i) {
        auto bit = reader.GetBit();
        if (i == 0 && bit == 0) {
            inverse = true;
        }
        real_val = (real_val << 1) + bit;
    }
    if (inverse) {
        real_val = real_val - (1 << cnt_bits) + 1;
    }
    coefs.push_back(real_val);
}

uint SOS::GetCntY(ImageData& img) {
    if (img.channels.size() == 1) {
        return 1;
    }
    return img.channels[1].h * img.channels[1].v;
}

std::vector<int> TransformToZigZag(std::vector<int>& vec) {
    std::vector<int> zz_vec(64, 0);
    int i = 0, j = 0, add = 1;
    for (size_t k = 0; k != vec.size(); ++k) {
        zz_vec[(i << 3) + j] = vec[k];
        if (i == 0 && j == 0) {
            ++j;
        } else if (i == 7 && add == 1) {
            ++j;
            add = -1;
        } else if (j == 7 && add == -1) {
            ++i;
            add = 1;
        } else if (j == 0 && add == 1) {
            ++i;
            add = -1;
        } else if (i == 0 && add == -1) {
            ++j;
            add = 1;
        } else {
            j -= add;
            i += add;
        }
    }
    return zz_vec;
}

uint GetMcuCnt(ImageData& img) {
    size_t mcu_width = 8;
    size_t mcu_height = 8;
    if (img.channels.size() > 1) {
        mcu_width *= img.channels[1].h;
        mcu_height *= img.channels[1].v;
    }

    uint mcu_line = (img.width + mcu_width - 1) / mcu_width;
    uint mcu_col = (img.height + mcu_height - 1) / mcu_height;

    return mcu_line * mcu_col;
}

void EOI::ProcessMarker(Reader& reader, ImageData& img) {
}

Image ImageCreator::Create() {
    Image img;

    img.SetSize(data_->width, data_->height);
    img.SetComment(data_->comment);

    size_t mcu_per_line;
    size_t mcu_per_col;
    CheckSizes(mcu_per_line, mcu_per_col);

    // only if there is cb and cr !!!!!!!
    // update: correct for all
    size_t mcu_width = 8;
    size_t mcu_height = 8;
    if (data_->channels.size() > 1) {
        mcu_width *= data_->channels[1].h;
        mcu_height *= data_->channels[1].v;
    }

    size_t i_mcu_id = 0, j_mcu_id = 0;
    size_t y_cnt = data_->mcu[0].y_matrix.size();
    size_t y_per_line = 1;
    if (data_->channels.size() > 1) {
        y_per_line = data_->channels[1].h;
    }
    for (int i = 0; i != data_->height; ++i) {
        for (int j = 0; j != data_->width; ++j) {
            size_t p = i % mcu_height;  // id in the mcu
            size_t q = j % mcu_width;

            size_t mcu_id_per_line = i / mcu_height;
            size_t mcu_id_per_col = j / mcu_width;

            size_t i_y_id = p >> 3;
            size_t j_y_id = q >> 3;
            size_t y_id = i_y_id * y_per_line + j_y_id;

            size_t mcu_id = mcu_id_per_line * mcu_per_line + mcu_id_per_col;

            size_t x = p % 8;
            size_t y = q % 8;

            if (data_->channels.size() > 1) {
                size_t h = data_->channels[1].h;
                size_t v = data_->channels[1].v;

                size_t y_coord = (x << 3) + y;
                size_t c_coord = (std::min(static_cast<int>(p / v), 8) << 3) +
                                 std::min(static_cast<int>(q / h), 8);
                // divide /v /h

                img.SetPixel(i, j,
                             ToRGB(data_->mcu[mcu_id].y_matrix[y_id][y_coord],
                                   data_->mcu[mcu_id].cb_matrix[c_coord],
                                   data_->mcu[mcu_id].cr_matrix[c_coord]));

            } else {
                size_t y_coord = (x << 3) + y;
                img.SetPixel(i, j, ToRGB(data_->mcu[mcu_id].y_matrix[y_id][y_coord], 128, 128));
            }
        }
    }
    return img;
}

void ImageCreator::CheckSizes(size_t& line, size_t& col) {
    // correct if there is cb and cr
    // update: correct for all
    size_t mcu_width = 8;
    size_t mcu_height = 8;
    if (data_->channels.size() > 1) {
        mcu_width *= data_->channels[1].h;
        mcu_height *= data_->channels[1].v;
    }
    line = (data_->width + mcu_width - 1) / mcu_width;
    col = (data_->height + mcu_height - 1) / mcu_height;
}

RGB ImageCreator::ToRGB(int y, int cb, int cr) {
    RGB rgb;

    rgb.r = round(y + 1.402 * (cr - 128));
    rgb.g = round(y - 0.34414 * (cb - 128) - 0.71414 * (cr - 128));
    rgb.b = round(y + 1.772 * (cb - 128));

    rgb.r = std::min(std::max(0, rgb.r), 255);
    rgb.g = std::min(std::max(0, rgb.g), 255);
    rgb.b = std::min(std::max(0, rgb.b), 255);
    return rgb;
}
