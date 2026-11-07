// The pip LibTorch ABI can differ from Ubuntu OpenCV's std::string ABI.
// Keep path strings behind a C boundary; cv::Mat has the same layout on both sides.
#include <opencv2/imgcodecs.hpp>
extern "C" bool gs_read_image(const char* path, cv::Mat* image) {
    try { *image = cv::imread(path); return !image->empty(); }
    catch (...) { return false; }
}
extern "C" bool gs_write_image(const char* path, const cv::Mat* image) {
    try { return cv::imwrite(path, *image); }
    catch (...) { return false; }
}
