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

#include "surfEntry.h"
#include <cv.h>
#include <stdlib.h>
#include <iostream>
#include <string>

using namespace std;
//=================================================================================================
// Constructor
//  This is the default "empty" constructor
//=================================================================================================
CSurfEntry::CSurfEntry() :
  mIPoints(),
  mLabelId(0),
  mName(),
  mMatchIndex(),
  mMatchCount(0),
  mThreshold(0.65f)
{
}

//=================================================================================================
// Destructor
//=================================================================================================
CSurfEntry::~CSurfEntry()
{
}

//=================================================================================================
//=================================================================================================
void CSurfEntry::Populate(IplImage* Image, unsigned LabelId, SSurfParams& SurfParams, string Name)
{
  surfDetDes(Image,
              mIPoints,
              SurfParams.mUpright,
              SurfParams.mOctaves,
              SurfParams.mIntervals,
              SurfParams.mInitSample,
              SurfParams.mThreshold);

  mLabelId = LabelId;
  mName = Name;

  mMatchIndex.resize(mIPoints.size());
}

//=================================================================================================
//=================================================================================================
void CSurfEntry::Populate(ifstream& Is, unsigned LabelId)
{
  //AsciiRead(Is);
  BinaryRead(Is);
  mLabelId = LabelId;

  mMatchIndex.resize(mIPoints.size());
}

//=================================================================================================
//=================================================================================================
const IpVec& CSurfEntry::GetDescriptor() const
{
  return mIPoints;
}

//=================================================================================================
//=================================================================================================
void CSurfEntry::Compare(CSurfEntry &Entry) // Fast version
{
  mMatchIndex.assign(mMatchIndex.size(), -1);
  mMatchCount = 0;

  float dist, d1, d2;
  float ratio;

  unsigned MatchIndex;

  //These are the SURF descriptors (ipoints) from the image to compare
  const IpVec IPointsToCompare = Entry.GetDescriptor();

  //Iterate through each SURF descriptor in current image
  for(unsigned i = 0; i < mIPoints.size(); i++)
  {
    d1 = d2 = FLT_MAX;

    //Compare against each SURF descriptor in input image
    for(unsigned j = 0; j < IPointsToCompare.size(); j++)
    {
      dist = mIPoints[i] - IPointsToCompare[j];

      if(dist<d1) // if this feature matches better than current best
      {
        d2 = d1;
        d1 = dist;
        MatchIndex = j;
      }
      else if(dist<d2) // this feature matches better than second best
      {
        d2 = dist;
      }
    }
    ratio = d1/d2;

    // If match has a d1:d2 ratio < 0.65 ipoints are a match
    if (ratio < mThreshold)
    {
      mMatchCount++;
      mMatchIndex.at(i) = MatchIndex;
    }
  }
}

//=================================================================================================
//=================================================================================================
void CSurfEntry::Compare(CSurfEntry &Entry, ofstream& Os) // The debug version
{
  mMatchIndex.assign(mMatchIndex.size(), -1);
  mMatchCount = 0;

  float dist, d1, d2;
  float ratio;

  unsigned MatchIndex;

  //These are the SURF descriptors (ipoints) from the image to compare
  const IpVec IPointsToCompare = Entry.GetDescriptor();

  Os.precision(2);
  Os << "<table border='0' cellpadding='1' cellspacing='3'>\n";
  Os << "<tr><td><b>Idx</b></td>\n";
  Os << "<td><b>D1</b></td>\n";
  Os << "<td><b>D2</b></td>\n";
  Os << "<td><b>D1/D2</b></td>\n";
  Os << "<td><b>Match</b></td></tr>\n";

  //Iterate through each SURF descriptor in current image
  for(unsigned i = 0; i < mIPoints.size(); i++)
  {
    Os << "<tr><td><b>" << i << "</b></td>\n";

    d1 = d2 = FLT_MAX;

    // Compare against each SURF descriptor in input image
    for(unsigned j = 0; j < IPointsToCompare.size(); j++)
    {
      dist = mIPoints[i] - IPointsToCompare[j];

      if(dist<d1) // if this feature matches better than current best
      {
        d2 = d1;
        d1 = dist;
        MatchIndex = j;
      }
      else if(dist<d2) // this feature matches better than second best
      {
        d2 = dist;
      }
    }
    ratio = d1/d2;

    // If match has a d1:d2 ratio < 0.65 ipoints are a match
    if (ratio < mThreshold)
    {
      mMatchCount++;
      mMatchIndex.at(i) = MatchIndex;
      Os << "<td><b>" << d1 << "</b></td>\n";
      Os << "<td><b>" << d2 << "</b></td>\n";
      Os << "<td><b>" << ratio << "</b></td>\n";
      Os << "<td><b>Yes</b></td>\n";
    }
    else
    {
      Os << "<td>" << d1 << "</td>\n";
      Os << "<td>" << d2 << "</td>\n";
      Os << "<td>" << ratio << "</td>\n";
      Os << "<td> No</td>\n";
    }
    Os << "</tr>\n";
  }
  Os << "</table>\n";
}

//=================================================================================================
// Description:
//  Returns the number of SURF descriptors that were matched when the compare function was called
//=================================================================================================
unsigned CSurfEntry::GetMatchCount() const
{
  return mMatchCount;
}

//=================================================================================================
//=================================================================================================
unsigned CSurfEntry::GetLabelId() const
{
  return mLabelId;
}

//=================================================================================================
//=================================================================================================
string CSurfEntry::GetName() const
{
  return mName;
}

//=================================================================================================
//=================================================================================================
unsigned CSurfEntry::GetDescriptorCount() const
{
  return mIPoints.size();
}

//=================================================================================================
//=================================================================================================
void CSurfEntry::SetClusterLabel(unsigned desIdx, unsigned clusterIdx)
{
  if (desIdx < mIPoints.size())
  {
    mIPoints.at(desIdx).clusterLabel = clusterIdx;
  }
}

/*
//=================================================================================================
//=================================================================================================
void CSurfEntry::SetClusterLabels(Mat& labels, unsigned offset, unsigned groupCount)
{
  for (unsigned i = 0; i < mIPoints.size(); i++)
  {
    unsigned label = labels.at<unsigned>(offset+i,0)
    mIPoints.at(i).clusterLabel = label;
  }
}
*/

//=================================================================================================
//=================================================================================================
unsigned CSurfEntry::GetClusterLabel(unsigned desIdx) const
{
  if (desIdx < mIPoints.size())
  {
    return mIPoints.at(desIdx).clusterLabel;
  }
  return 0;
}

//=================================================================================================
//=================================================================================================
bool CSurfEntry::AsciiRead(ifstream& Is)
{
  int count;

  // clear the ipts vector first
  mIPoints.clear();

  // read descriptor length/number of ipoints
  Is >> mName;
  Is >> count;

  // for each ipoint
  for (int i = 0; i < count; i++) 
  {
    Ipoint ipt;

    // read vals
    Is >> ipt.scale; 
    Is >> ipt.x;
    Is >> ipt.y;
    Is >> ipt.orientation;
    Is >> ipt.laplacian;

    // read descriptor components
    for (int j = 0; j < 64; j++)
      Is >> ipt.descriptor[j];

    mIPoints.push_back(ipt);
  }
  return true;
}

//=================================================================================================
//=================================================================================================
bool CSurfEntry::AsciiWrite(ofstream& Os)
{
  // output descriptor length
  Os << mName << " ";
  Os << mIPoints.size() << "\n";

  // create output line as:  scale  x  y  des
  for(unsigned i=0; i < mIPoints.size(); i++)
  {
    Os << mIPoints.at(i).scale << "  ";
    Os << mIPoints.at(i).x << " ";
    Os << mIPoints.at(i).y << " ";
    Os << mIPoints.at(i).orientation << " ";
    Os << mIPoints.at(i).laplacian << " ";

    for(unsigned j=0; j<64; j++)
      Os << mIPoints.at(i).descriptor[j] << " ";

    Os << "\n";
  }
  return true;
}

//=================================================================================================
// Description:
//  Reads the Surf entry from a binary file.
//  IMPORTANT: make sure the input file stream is opened with the ios::binary flag
//=================================================================================================
bool CSurfEntry::BinaryRead(ifstream& Is)
{
  unsigned nameCount = 0;
  Is.read(reinterpret_cast<char*>(&nameCount), sizeof(nameCount));
  mName.resize(nameCount);

  for(unsigned i=0; i < nameCount; i++)
  {
    char character;
    Is.read(reinterpret_cast<char*>(&character), sizeof(character));
    mName.at(i) = character;
  }

  // clear the ipts vector first
  mIPoints.clear();

  int count;
  Is.read(reinterpret_cast<char*>(&count), sizeof(count));

  // for each ipoint
  for (int i = 0; i < count; i++) 
  {
    Ipoint ipt;

    Is.read(reinterpret_cast<char*>(&ipt.scale),       sizeof(ipt.scale));
    Is.read(reinterpret_cast<char*>(&ipt.x),           sizeof(ipt.x));
    Is.read(reinterpret_cast<char*>(&ipt.y),           sizeof(ipt.y));
    Is.read(reinterpret_cast<char*>(&ipt.orientation), sizeof(ipt.orientation));
    Is.read(reinterpret_cast<char*>(&ipt.laplacian),   sizeof(ipt.laplacian));

    for (int j = 0; j < 64; j++)
      Is.read(reinterpret_cast<char*>(&(ipt.descriptor[j])), sizeof(ipt.descriptor[j]));

    mIPoints.push_back(ipt);
  }
  return true;
}

//=================================================================================================
// Description:
//  Writes the Surf entry to a binary file.
//  IMPORTANT: make sure the output file stream is created with the ios::binary flag
//=================================================================================================
bool CSurfEntry::BinaryWrite(ofstream& Os)
{
  unsigned nameCount = mName.size();
  Os.write(reinterpret_cast<char*>(&nameCount), sizeof(nameCount));

  for(unsigned i=0; i < nameCount; i++)
  {
    char character = mName.at(i);
    Os.write(reinterpret_cast<const char*>(&character), sizeof(character));
  }

  unsigned count = mIPoints.size();
  Os.write(reinterpret_cast<char*>(&count), sizeof(count));

  for(unsigned i=0; i < mIPoints.size(); i++)
  {

    //Assign shorter names for readability
    float& scale       = mIPoints.at(i).scale;
    float& x           = mIPoints.at(i).x;
    float& y           = mIPoints.at(i).y;
    float& orientation = mIPoints.at(i).orientation;
    int& laplacian     = mIPoints.at(i).laplacian;

    //Type& Value
    Os.write(reinterpret_cast<char*>(&scale),       sizeof(scale));
    Os.write(reinterpret_cast<char*>(&x),           sizeof(x));
    Os.write(reinterpret_cast<char*>(&y),           sizeof(y));
    Os.write(reinterpret_cast<char*>(&orientation), sizeof(orientation));
    Os.write(reinterpret_cast<char*>(&laplacian),   sizeof(laplacian));

    for(unsigned j = 0; j < 64; j++)
      Os.write(reinterpret_cast<char*>(&(mIPoints.at(i).descriptor[j])),
        sizeof(mIPoints.at(i).descriptor[j]));

  }
  return true;
}

//=================================================================================================
//=================================================================================================
void CSurfEntry::WriteHtmlInfo(ofstream& Os)
{
  int EntryPerColumn = (int)(mIPoints.size()/6.0f+0.5f);
  Os << "<b>Name: </b>" << mName << "<br>\n";
  Os << "<b>LabelId: </b>" << mLabelId << "<br>\n";
  Os << "<b>Descriptors: </b>" << mIPoints.size() << "<br>\n";
  Os << "<b>Size: </b>" << mIPoints.size()*sizeof(Ipoint) << " bytes <br><br>\n";
  Os << "<table border='0' cellpadding='2' cellspacing='2'>\n";
  Os << "<tr><td valign='top'>\n";

  unsigned EntryInColumn = 0;

  // create output line as:  scale  x  y  des
  for(unsigned i=0; i < mIPoints.size(); i++)
  {
    if (EntryInColumn == EntryPerColumn)
    {
      EntryInColumn = 0;
      Os << "</td><td valign='top'>\n";
    }
    Os << "<table border='0' cellpadding='1' cellspacing='1'>\n";
    Os << "<tr><td><b>Descriptor: </b></td><td>"  << i                           << "</td></tr>\n";
    Os << "<tr><td><b>Scale: </b></td><td>"       << mIPoints.at(i).scale        << "</td></tr>\n";
    Os << "<tr><td><b>X: </b></td><td>"           << (int)mIPoints.at(i).x       << "</td></tr>\n";
    Os << "<tr><td><b>Y: </b></td><td>"           << (int)mIPoints.at(i).y       << "</td></tr>\n";
    Os << "<tr><td><b>Orientation: </b></td><td>" << mIPoints.at(i).orientation  << "</td></tr>\n";
    Os << "<tr><td><b>Laplacian: </b></td><td>"   << mIPoints.at(i).laplacian    << "</td></tr>\n";
    Os << "<tr><td><b>Cluster: </b></td><td>"     << mIPoints.at(i).clusterLabel << "</td></tr>\n";
    Os << "</table>\n<br>\n";
    EntryInColumn++;
  }
  Os << "</tr></td>\n";
  Os << "</table>\n<br><br><br>\n";
}
