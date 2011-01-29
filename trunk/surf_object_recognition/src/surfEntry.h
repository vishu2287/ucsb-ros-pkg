//=================================================================================================
// Copyright (c) 2011, Paul Filitchkin
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the UCSB Robotics Lab nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL PAUL FILITCHKIN BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

//=================================== Class Description ===========================================
//  This class primarily stores the SURF descriptor for a single image.  It also stores various
// identification information such as the image label, filename, etc.
//=================================================================================================

#ifndef SURF_ENTRY_H
#define SURF_ENTRY_H

#include <string>
#include <map>
#include <fstream>
#include "surflib.h"

class CSurfEntry
{
  public:

    struct SSurfParams
    {
      bool mUpright;
      int mOctaves;
      int mInitSample;
      int mIntervals;
      float mThreshold;
    };

    CSurfEntry();
    ~CSurfEntry();

    void Populate(
      IplImage* Image,
      unsigned LabelId,
      SSurfParams& SurfParams,
      std::string Name );

    void Populate(std::ifstream& Is, unsigned LabelId);
    void Compare(CSurfEntry& entry);
    void Compare(CSurfEntry& Entry, std::ofstream& Os);
    const IpVec& GetDescriptor() const;
    unsigned GetMatchCount() const;
    unsigned GetLabelId() const;
    std::string GetName() const;
    unsigned GetDescriptorCount() const;
    void SetClusterLabel(unsigned desIdx, unsigned clusterIdx);
    unsigned GetClusterLabel(unsigned desIdx) const;

    bool BinaryWrite( std::ofstream& Os );
    bool BinaryRead( std::ifstream& Is );
    bool AsciiRead( std::ifstream& Is );
    bool AsciiWrite( std::ofstream& Os );
    void WriteHtmlInfo( std::ofstream& Os );

    //Stores the index of the best match (that is the index of the ipoint) from the image we
    // are comparing with
    std::vector<int> mMatchIndex;

  private:
    IpVec mIPoints; // The SURF descriptors
    unsigned mLabelId; //The ID that corresponds to the entries label string
                       // The set of label strings is stored in the database
    std::string mName; //The unique name of the entry (same as the image 
                       // filename without the extension)
    double mThreshold; //The ratio threshold used to match entries

    unsigned mMatchCount;

};

#endif //end #ifndef SURF_ENTRY_H
