#include <imageprocext.h>
#include <QDebug>
#include <opencv2/highgui.hpp>
namespace imageprocext
{
Mat unsharpMasking(const Mat& frame, double coeff)
{ 
    Mat bluredBright, image;
    std::vector <Mat> channels;
    split(frame, channels);
    GaussianBlur(channels[2], bluredBright, Size(0, 0), 5);
    addWeighted(channels[2], coeff + 1, bluredBright, -coeff, 0, channels[2]);
    merge(channels, image);
    return image;
}

Mat gammaCorrection(const Mat& frame, double coeff)
{
    Mat lookUpTable(1, 256, CV_8U);
    uchar* p = lookUpTable.ptr();
    for( int i = 0; i < 256; ++i)
        p[i] = saturate_cast<uchar>(pow((double)i / 255.0, coeff) * 255.0);
    Mat res = frame.clone();
    LUT(frame, lookUpTable, res);
    return res;
}

Mat gain(const Mat& frame, double coeff, double add)
{
    Mat new_image = Mat::zeros( frame.size(), frame.type() );
    for (int y = 0; y < frame.rows; y++)
    {
        for (int x = 0; x < frame.cols; x++)
        {
            for (int c = 0; c < frame.channels(); c++)
            {
                new_image.at<Vec3b>(y,x)[c] =
                        saturate_cast<uchar>( coeff * (double)frame.at<Vec3b>(y,x)[c] + add );
            }
        }
    }
    return new_image;
}

Mat gainChannel(int channel, const Mat &frame, double coeff, double add)
{
    std::vector <Mat> channels;
    split(frame, channels);
    for (int y = 0; y < frame.rows; y++)
    {
        for (int x = 0; x < frame.cols; x++)
        {
            channels[channel].at<uchar>(y,x) =
                    saturate_cast<uchar>( coeff * (double)channels[channel].at<uchar>(y,x) + add );

        }
    }
    Mat resultImage;
    merge(channels, resultImage);
    return resultImage;
}

Mat equalizeHistogramm(const Mat& frame)
{
    
    std::vector <Mat> channels;
    split(frame, channels);
    
    equalizeHist(channels[2], channels[2]);
    
    Mat equalized;
    merge(channels, equalized);
    
    return equalized;
}

AutoExposureHandler::AutoExposureHandler(QObject *parent) : QObject(parent)
{

}

bool AutoExposureHandler::correct(Mat image)
{
    int bins = 256;
    int histSize[] = {bins};
    // Set ranges for histogram bins
    float lranges[] = {0, 256};
    const float* ranges[] = {lranges};
    // create matrix for histogram
    cv::Mat hist;
    int channels[] = {0}; // hsv
    
    calcHist( &image, 1, channels, Mat(), // do not use mask
              hist, 1, histSize, ranges);
    normalize(hist, hist, 1, 0, NORM_L1);

    percent = hist.at <float> (255.0);

    double mean = cv::mean(image)[0];
    double relMean = mean / params->mean();
    double rel = percent / params->max_percent();
    if (relMean > 1 && rel > 1)
    {
        rel = relMean > rel ? relMean : rel;
    }
    else
    {
        rel = relMean;
    }
    //qDebug() << relMean << rel << percent << params->max_percent() << mean << params->mean();
    if (!(rel <= params->max_rel_coef() && rel >= params->min_rel_coef()))
    {
        ++processCounter;
        if (processCounter > maxFrameCoeff && divideCoeff < divideCoeffMax)
        {
            divideCoeff += 2;
            processCounter = 0;
        }
        if (processCounter > maxFrameCoeff * 3 && divideCoeff == divideCoeffMax)
        {
            divideCoeff = divideCoeffDefault;
        }
        double neededExposure = params->exposure() / rel;
        double exposureStep = neededExposure - params->exposure();
        double gainStep = (exposureStep / divideCoeff) * 15;
        gainStep = gainStep < 1 ? 1 : gainStep;

        params->set_exposure(params->exposure() + exposureStep / divideCoeff);

        if (params->exposure() > lowExp && params->gain() <= params->max_gain_coeff())
        {
            params->set_gain(params->gain() + gainStep);
        }
        else if (params->exposure() < lowExp)
        {
            params->set_gain(params->min_gain_coeff());
        }
        //qDebug()<< neededExposure << exposureStep << gainStep << divideCoeff;

        if (params->exposure() > maxExp)
        {
            params->set_exposure(maxExp);
        }
        if (params->gain() > params->max_gain_coeff())
        {
            params->set_gain(params->max_gain_coeff());
        }

        //        emit currentStateReady(QString("AUTOEXP : count max pixel: %1, new exp: %2, maxPer: %3, gain: %4")
        //                               .arg(percent)
        //                               .arg(params->exposure())
        //                               .arg(params->max_percent())
        //                               .arg(params->gain()));
        return true;
    }
    else
    {
        processCounter = 0;
        return false;
    }
    
}


Mat shadowing(const Mat& frame, int thres, int windowSize, double coeff)
{
    std::vector <Mat> channels;
    split(frame, channels);
    Mat brightCopy;
    channels[2].copyTo(brightCopy);

    for (int i = 0; i < brightCopy.rows; ++i)
    {
        for (int j = 0; j < brightCopy.cols; j++)
        {
            if (brightCopy.at <uchar> (i, j) > thres)
            {
                brightCopy.at <uchar> (i, j) = 255;
            }
        }
    }

    GaussianBlur(brightCopy, brightCopy, Size(0, 0), windowSize);

    bitwise_not ( brightCopy, brightCopy );

    addWeighted(channels[2], 1, brightCopy, coeff, 0, channels[2]);

    Mat result;
    merge(channels, result);
    //cvtColor(result, result, CV_HSV2RGB);

    return result;
}

}
