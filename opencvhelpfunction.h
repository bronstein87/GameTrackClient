#ifndef OPENCVHELPFUNCTION_H
#define OPENCVHELPFUNCTION_H
#include <opencv2/core.hpp>
#include <QtGlobal>
using namespace cv;

template <typename Rect>
void checkRectangleSize(Rect& r, qint32 width, qint32 height)
{
    if (r.x >= width || r.y >= height)
    {
        r = Rect();
    }
    else
    {
        r.x = r.x < 0 ? 0 : r.x;
        r.y = r.y < 0 ? 0 : r.y;

        r.width = r.width + r.x > width ? width - r.x : r.width;
        r.height = r.height + r.y > height ? height - r.y : r.height;
    }
}

#endif // OPENCVHELPFUNCTION_H
