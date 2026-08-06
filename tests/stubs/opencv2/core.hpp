#pragma once
/**
 * Minimal OpenCV stub used ONLY by hardware-independent logic tests
 * (ICAR_LOGIC_TESTS_ONLY). It provides just enough types and functions for
 * the production FSM/control sources to compile and link without OpenCV.
 * It is intentionally tiny and never used in the real-car build.
 */

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <string>
#include <vector>

namespace cv
{
enum
{
    CV_8U = 0,
    CV_32F = 5,
    CV_64F = 6,
    CV_8UC3 = 16,
    FONT_HERSHEY_PLAIN = 0,
    FONT_HERSHEY_TRIPLEX = 3,
    FILLED = -1,
    INTER_LINEAR = 1,
    CV_INTER_LINEAR = 1
};

typedef unsigned char uchar;

#define CV_RGB(r, g, b) cv::Scalar((b), (g), (r))

struct Size
{
    int width{0};
    int height{0};
    Size() = default;
    Size(int w, int h) : width(w), height(h) {}
};

struct Point2d;
struct Point2f;

struct Point
{
    int x{0};
    int y{0};
    Point() = default;
    Point(int px, int py) : x(px), y(py) {}
    Point(const Point2f &other);
    Point(const Point2d &other);
};

struct Point2f
{
    float x{0.0f};
    float y{0.0f};
    Point2f() = default;
    Point2f(float px, float py) : x(px), y(py) {}
    Point2f(const Point2d &other);
};

struct Point2d
{
    double x{0.0};
    double y{0.0};
    Point2d() = default;
    Point2d(double px, double py) : x(px), y(py) {}
    Point2d(const Point2f &other);
};

inline Point2f::Point2f(const Point2d &other)
    : x(static_cast<float>(other.x)), y(static_cast<float>(other.y))
{
}

inline Point2d::Point2d(const Point2f &other) : x(other.x), y(other.y) {}

inline Point::Point(const Point2f &other)
    : x(static_cast<int>(other.x)), y(static_cast<int>(other.y))
{
}

inline Point::Point(const Point2d &other)
    : x(static_cast<int>(other.x)), y(static_cast<int>(other.y))
{
}

inline Point2d operator+(const Point2d &left, const Point2d &right)
{
    return {left.x + right.x, left.y + right.y};
}

struct Point3d
{
    double x{0.0};
    double y{0.0};
    double z{0.0};
    Point3d() = default;
    Point3d(double px, double py, double pz) : x(px), y(py), z(pz) {}
};

struct Rect
{
    int x{0};
    int y{0};
    int width{0};
    int height{0};
    Rect() = default;
    Rect(int px, int py, int w, int h) : x(px), y(py), width(w), height(h) {}
};

struct Scalar
{
    double v[4]{0.0, 0.0, 0.0, 0.0};
    Scalar() = default;
    Scalar(double v0, double v1, double v2, double v3 = 0.0)
        : v{v0, v1, v2, v3} {}
};

class Mat
{
public:
    int rows{0};
    int cols{0};

    Mat() = default;

    template <typename T>
    T &at(int row, int column)
    {
        assert(row >= 0 && row < rows && column >= 0 && column < cols);
        return *reinterpret_cast<T *>(
            data_.data() + (static_cast<std::size_t>(row) * cols + column) *
                               elemSize_);
    }

    template <typename T>
    const T &at(int row, int column) const
    {
        assert(row >= 0 && row < rows && column >= 0 && column < cols);
        return *reinterpret_cast<const T *>(
            data_.data() + (static_cast<std::size_t>(row) * cols + column) *
                               elemSize_);
    }

    template <typename T>
    T *ptr(int row)
    {
        assert(row >= 0 && row < rows && elemSize_ == sizeof(T));
        return reinterpret_cast<T *>(
            data_.data() + static_cast<std::size_t>(row) * cols * elemSize_);
    }

    void create(Size size, int type)
    {
        rows = size.height;
        cols = size.width;
        type_ = type;
        elemSize_ = elemSizeForType(type);
        data_.assign(static_cast<std::size_t>(rows) * cols * elemSize_, 0);
    }

    Mat inv() const
    {
        Mat result;
        if (rows != 3 || cols != 3 || elemSize_ != sizeof(double))
            return result;
        const double a = at<double>(0, 0), b = at<double>(0, 1),
                     c = at<double>(0, 2);
        const double d = at<double>(1, 0), e = at<double>(1, 1),
                     f = at<double>(1, 2);
        const double g = at<double>(2, 0), h = at<double>(2, 1),
                     i = at<double>(2, 2);
        const double det = a * (e * i - f * h) - b * (d * i - f * g) +
                           c * (d * h - e * g);
        result.create(Size(3, 3), CV_64F);
        if (det == 0.0)
            return result;
        const double invDet = 1.0 / det;
        result.at<double>(0, 0) = (e * i - f * h) * invDet;
        result.at<double>(0, 1) = (c * h - b * i) * invDet;
        result.at<double>(0, 2) = (b * f - c * e) * invDet;
        result.at<double>(1, 0) = (f * g - d * i) * invDet;
        result.at<double>(1, 1) = (a * i - c * g) * invDet;
        result.at<double>(1, 2) = (c * d - a * f) * invDet;
        result.at<double>(2, 0) = (d * h - e * g) * invDet;
        result.at<double>(2, 1) = (b * g - a * h) * invDet;
        result.at<double>(2, 2) = (a * e - b * d) * invDet;
        return result;
    }

    bool empty() const { return rows == 0 || cols == 0; }
    int type() const { return type_; }

    static Mat zeros(Size size, int type)
    {
        Mat result;
        result.create(size, type);
        std::fill(result.data_.begin(), result.data_.end(), 0);
        return result;
    }

    static std::size_t elemSizeForType(int type)
    {
        switch (type)
        {
        case CV_8UC3: return 3;
        case CV_32F: return 4;
        case CV_64F: return 8;
        default: return 1;
        }
    }

private:
    int type_{CV_8U};
    std::size_t elemSize_{1};
    std::vector<unsigned char> data_;
};

inline Mat getPerspectiveTransform(const std::vector<Point2f> &,
                                   const std::vector<Point2f> &)
{
    Mat identity;
    identity.create(Size(3, 3), CV_64F);
    identity.at<double>(0, 0) = 1.0;
    identity.at<double>(1, 1) = 1.0;
    identity.at<double>(2, 2) = 1.0;
    return identity;
}

inline void remap(const Mat &, Mat &, const Mat &, const Mat &, int, int = 0,
                  const Scalar & = Scalar())
{
}

inline bool imwrite(const std::string &, const Mat &) { return true; }

inline void line(Mat &, Point, Point, const Scalar &, int = 1, int = 8,
                 int = 0)
{
}

inline void circle(Mat &, Point, int, const Scalar &, int = 1, int = 8,
                   int = 0)
{
}

inline void rectangle(Mat &, Rect, const Scalar &, int = 1, int = 8, int = 0)
{
}

inline void putText(Mat &, const std::string &, Point, int, double,
                    const Scalar &, double = 1.0, int = 8, bool = false)
{
}

} // namespace cv
