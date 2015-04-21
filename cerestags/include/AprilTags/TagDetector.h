#ifndef TAGDETECTOR_H
#define TAGDETECTOR_H

#include <vector>

#include "opencv2/opencv.hpp"

#include "AprilTags/TagDetection.h"
#include "AprilTags/TagFamily.h"
#include "AprilTags/FloatImage.h"
#include "AprilTags/Tag64.h"

namespace AprilTags {

class TagDetector {
public:
	
	const TagFamily thisTagFamily;
	const TagFamily annTagFamily;
	const bool isUsingAnn;

	//! Constructor
  // note: TagFamily is instantiated here from TagCodes
	TagDetector(const TagCodes& tagCodes, const TagCodes& annTagCodes): 
		thisTagFamily(tagCodes),
		annTagFamily(annTagCodes),
		isUsingAnn(true)
		{}

	TagDetector(const TagCodes& tagCodes): 
		thisTagFamily(tagCodes),
		annTagFamily(tagCodes64),
		isUsingAnn(false)
		{}
	
	std::vector<TagDetection> extractTags(const cv::Mat& image);
	
};

} // namespace

#endif
