#pragma once

namespace AprilTags {

const unsigned long long tag64[] =
  { 
    0x55800180018001AALL
  };

static const TagCodes tagCodes64 = TagCodes(64, 15, tag64, sizeof(tag64)/sizeof(tag64[0]));

}
