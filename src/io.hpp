#pragma once
#include <random>
#include <Eigen/Dense>

#include <cereal/cereal.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/list.hpp>

namespace cereal
{
  template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols> inline
    typename std::enable_if<traits::is_output_serializable<BinaryData<_Scalar>, Archive>::value, void>::type
    save(Archive& ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> const& m)
  {
    int32_t rows = m.rows();
    int32_t cols = m.cols();
    ar(rows);
    ar(cols);
    ar(binary_data(m.data(), rows * cols * sizeof(_Scalar)));
  }

  template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols> inline
    typename std::enable_if<traits::is_input_serializable<BinaryData<_Scalar>, Archive>::value, void>::type
    load(Archive& ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& m)
  {
    int32_t rows;
    int32_t cols;
    ar(rows);
    ar(cols);

    m.resize(rows, cols);

    ar(binary_data(m.data(), static_cast<std::size_t>(rows * cols * sizeof(_Scalar))));
  }
}

class ImageTensor
{
public:
  ImageTensor(uint height_, uint width_, uint channels_) : height(height_), width(width_), channels(channels_), pixels(height_* width_* channels_, 0) {};
  ImageTensor(const ImageTensor& other) : height(other.height), width(other.width), channels(other.channels), pixels(other.pixels) {};
  ImageTensor() {};
  float& operator()(uint i, uint j, uint k)
  {
    return pixels[i * channels * width + j * channels + k];
  };
  ImageTensor& operator=(const ImageTensor& other)
  {
    height = other.height;
    width = other.width;
    channels = other.channels;
    pixels = other.pixels;
    return *this;
  };

  std::vector<float> pixels;
  uint height, width, channels;
};