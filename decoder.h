#pragma once

#include "utils/image.h"
#include "huffman.h"
#include "fft.h"
#include <istream>
#include <string>
#include <memory>
#include <map>
#include <vector>

Image Decode(std::istream& input);

struct ChannelInfo {
    uint h;
    uint v;
    uint quant_table_id;
    uint dc_table_id;
    uint ac_table_id;
};

struct MCU {
    std::vector<std::vector<int>> y_matrix;
    std::vector<int> cb_matrix;
    std::vector<int> cr_matrix;
};

struct ImageData {
    uint height;
    uint width;
    std::string comment;
    std::map<int, std::vector<uint>> dqt_tables;
    std::vector<std::map<int, HuffmanTree>> dht_tables;  // dht[0] dc, dht[1] ac, dht[][table_id]
    std::vector<ChannelInfo> channels;                   // 0-y 1-cb 2-cr
    uint precision;
    std::vector<MCU> mcu;
    bool sof0_check = false;
    bool sos_check = false;
};

void Quantization(ImageData& img);
void IDCTTransformation(ImageData& img);

class Reader {
public:
    Reader(std::istream* input) : in_(input), eof_(false) {
        bit_buff_.resize(0);
    }
    void Process(ImageData& img);
    void NextMarker();
    unsigned int GetByte();
    uint GetBit();
    bool IfLastWasFF() const {
        return last_was_ff_;
    }
    bool IsEOF();
    bool NextIsMarker();

private:
    void ReadBits();
    std::vector<int> bit_buff_;
    bool last_was_ff_ = false;
    std::istream* in_;
    std::string next_marker_;
    bool eof_;
};

class Marker : public std::enable_shared_from_this<Marker> {
public:
    virtual void ProcessMarker(Reader& reader, ImageData& img) {
    }
    virtual ~Marker() = default;
};

class SOF0 : public Marker {
public:
    void ProcessMarker(Reader& reader, ImageData& img) override;
};

class DHT : public Marker {
public:
    void ProcessMarker(Reader& reader, ImageData& img) override;
};

class DQT : public Marker {
public:
    void ProcessMarker(Reader& reader, ImageData& img) override;
};

class APPn : public Marker {
public:
    void ProcessMarker(Reader& reader, ImageData& img) override;
};

class COM : public Marker {
public:
    void ProcessMarker(Reader& reader, ImageData& img) override;
};

class SOS : public Marker {
public:
    void ProcessMarker(Reader& reader, ImageData& img) override;
    std::vector<int> GetMatrix(Reader& reader, ImageData& img, uint channel_id);
    void FillCoefs(Reader& reader, ImageData& img, int val, std::vector<int>& coefs);
    uint GetCntY(ImageData& img);
};

class EOI : public Marker {
public:
    void ProcessMarker(Reader& reader, ImageData& img) override;
};

struct MarkersTable {
    static std::map<std::string, std::shared_ptr<Marker>> markers;
};

std::vector<int> TransformToZigZag(std::vector<int>& vec);

class ImageCreator {
public:
    ImageCreator(ImageData* img) : data_(img) {
    }
    Image Create();
    void CheckSizes(size_t& line, size_t& col);
    RGB ToRGB(int y, int cb, int cr);

private:
    ImageData* data_;
};

uint GetMcuCnt(ImageData& img);