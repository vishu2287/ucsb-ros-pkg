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
//  This class holds a collection of SURF entries.  The constructor builds the database and 
// class functions are responsible for classification.
//=================================================================================================
#ifndef SURF_DATABASE_H
#define SURF_DATABASE_H

#include <string>
#include <map>
#include <vector>
#include "surfEntry.h"

class CvSVM;
struct CvParamGrid;

class CSurfDatabase
{
  public:

    struct SPaths
    {
      std::string mSetupPath;
      std::string mImagePath;
      std::string mDatabasePath;
      std::string mHtmlPath;
    };

    CSurfDatabase(std::string Name, SPaths Paths, bool trainClassifier=false);
    ~CSurfDatabase();

    unsigned MatchEntry(CSurfEntry& Entry,
                        std::string DbName,
                        std::ofstream& Os,
                        unsigned& BestMatchCount);

    bool MatchEntryNN(CSurfEntry& Entry, unsigned& LabelId, unsigned& BestMatchCount);

    //Returns the label string which maps to the input label ID (this is a one-to-one mapping)
    std::string GetLabelName(unsigned LabelId);

    //Returns the label ID which maps to the input label string (this is a one-to-one mapping)
    int GetLabelId(std::string Label);

    //Returns a reference to the ith SURF entry from the vector of entries
    CSurfEntry& GetSurfEntry(unsigned i);

    //Returns the size of the SURF entry vector
    unsigned GetEntryCount() const;
    void MatchDatabase(CSurfDatabase& Database);
    std::string& GetName();
    unsigned ClassifyEntrySVM(CSurfEntry& Entry);
    void ClassifyDatabase( CSurfDatabase& Database );

  private:

    void SetSVMTrainAutoParams(
      CvParamGrid& c_grid, CvParamGrid& gamma_grid,
      CvParamGrid& p_grid, CvParamGrid& nu_grid,
      CvParamGrid& coef_grid, CvParamGrid& degree_grid );

    void UpdateLabels(std::string ImageLabel);

    // Helper functions for reading setup parameters
    bool ReadUpright(std::ifstream& DbFile);
    unsigned ReadOcatves(std::ifstream& DbFile);
    unsigned ReadIntervals(std::ifstream& DbFile);
    unsigned ReadInitSample(std::ifstream& DbFile);
    float ReadThreshold(std::ifstream& DbFile);
    bool ReadAutoLevels(std::ifstream& DbFile);
    bool ReadGenerateHtml(std::ifstream& DbFile);
    void WriteEntryHtmlSummary(CSurfEntry& Entry, std::string HtmlPath);

    SPaths mPaths;
    std::string mName;

    // Parameters from the setup file that are not related to
    bool mGenerateHtml;
    bool mAutoLevels;

    CSurfEntry::SSurfParams mParams;

    std::vector<std::string> mLabels;
    std::vector<std::string> mImagePaths;
    std::vector<CSurfEntry> mSurfEntries;

    CvSVM* mpSVMClassifier;
    cv::Mat* mpCenters;
    cv::Mat* mpLabels;
    int mClusterCount;

};

#endif //end #ifndef SURF_DATABASE_H
