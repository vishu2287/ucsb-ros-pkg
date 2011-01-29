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

#include "surfDatabase.h"
#include <cv.h>
#include <ml.h>
#include <highgui.h>
#include "histLib.h"
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

//=================================================================================================
// Constructor
//=================================================================================================
CSurfDatabase::CSurfDatabase(string Name, SPaths Paths, bool trainClassifier)
{
  mPaths = Paths;
  string DbSetupFilePath = mPaths.mSetupPath + Name + ".txt";
  ifstream DbFile(DbSetupFilePath.c_str());
  string ImageName;
  string ImageLabel;
  string ImagePath;

  unsigned LabelId = 0;

  mName = Name;

  bool ValidEntry = false;

  string PrevImageLabel;

  if (DbFile.is_open())
  {
    mParams.mUpright    = ReadUpright(DbFile);
    mParams.mOctaves    = ReadOcatves(DbFile);
    mParams.mIntervals  = ReadIntervals(DbFile);
    mParams.mInitSample = ReadInitSample(DbFile);
    mParams.mThreshold  = ReadThreshold(DbFile);
    mAutoLevels         = ReadAutoLevels(DbFile);
    mGenerateHtml       = ReadGenerateHtml(DbFile);

    while (!DbFile.eof())
    {
      // Read the image filename
      getline(DbFile, ImageName);

      ImageName.erase(remove(ImageName.begin(), ImageName.end(), '\r'), ImageName.end());
      ImageName.erase(remove(ImageName.begin(), ImageName.end(), '\n'), ImageName.end());

      // Construct the full path
      ImagePath = mPaths.mImagePath + mName + "/" + ImageName;

      // Read the label string
      getline(DbFile, ImageLabel);

      ImageLabel.erase(remove(ImageLabel.begin(), ImageLabel.end(), '\r'), ImageLabel.end());
      ImageLabel.erase(remove(ImageLabel.begin(), ImageLabel.end(), '\n'), ImageLabel.end());

      string ImageNameNoExt = ImageName.substr(0, ImageName.size()-4);
      string DbEntryPath = mPaths.mDatabasePath + mName + "/" + ImageNameNoExt + ".sef";

      ifstream DbEntry(DbEntryPath.c_str(), ios::in|ios::binary);

      CSurfEntry NewEntry = CSurfEntry();

      // If there is already a saved entry read it from disk
      if (DbEntry)
      {
        // Update the labels and get new label ID
        UpdateLabels(ImageLabel);
        LabelId = mLabels.size()-1;

        // Load SURF decriptors from file stream
        NewEntry.Populate(DbEntry, LabelId);

        // Add the new SURF entry
        mSurfEntries.push_back(NewEntry);

        // Store the image path
        mImagePaths.push_back(ImagePath);
      }
      else // Otherwise populate an entry and save it
      {
        // Open the image
        IplImage *Image = cvLoadImage(ImagePath.c_str());

        // If we opened a valid image
        if (Image)
        {
          // Update the labels and get new label ID
          UpdateLabels(ImageLabel);
          LabelId = mLabels.size()-1;

          // Pointer to auto leveled image. In this scope because it is
          // potentially needed further down
          IplImage *ImageNorm;

          // Perform auto levels if the option was specified
          if (mAutoLevels)
          {
            ImageNorm = cvCreateImage(cvGetSize(Image), IPL_DEPTH_8U, 3);
            NormalizeClipImageBGR(Image, ImageNorm);

            // Compute the SURF descriptors from the level adjusted image
            NewEntry.Populate(ImageNorm, LabelId, mParams, ImageNameNoExt);
          }
          else
          {
            // Compute the SURF descriptors from the original image
            NewEntry.Populate(Image, LabelId, mParams, ImageNameNoExt);
          }

          // Add the new SURF entry to the collection in this database
          mSurfEntries.push_back(NewEntry);

          // Save the SURF descriptors we just generated
          ofstream DbEntrySave(DbEntryPath.c_str(), ios::out|ios::binary);
          NewEntry.BinaryWrite(DbEntrySave);

          // Generate html files after we generate new descriptors
          if (mGenerateHtml)
          {
            mImagePaths.push_back(ImagePath);
            string HtmlImagePath = mPaths.mHtmlPath + mName + "/" + ImageName;

            //================ Generate and save the necessary images for html ====================
            const int params[3] = {CV_IMWRITE_JPEG_QUALITY, 100, 0};

            // Generate color histogram for the current image
            CvSize HistSize = cvSize(2*HIST_EDGE + 3*HIST_BINS, 2*HIST_EDGE + HIST_HEIGHT);
            IplImage *HistImg = cvCreateImage(HistSize , IPL_DEPTH_8U, 3);
            DrawHistBar(HistImg);
            string HistImageName = ImageNameNoExt + ".hist.png";
            string HtmlHistPath = mPaths.mHtmlPath + mName + "/" + HistImageName;

            // If auto levels was performed
            if (mAutoLevels)
            {
              DrawHistogramBGR(ImageNorm, HistImg);
              cvSaveImage(HtmlHistPath.c_str(), HistImg);

              drawIpoints(ImageNorm, NewEntry.GetDescriptor());
              cvSaveImage(HtmlImagePath.c_str(), ImageNorm, params);
              cvReleaseImage(&ImageNorm);
              cvReleaseImage(&HistImg);
            }
            else
            {
              DrawHistogramBGR(Image, HistImg);
              cvSaveImage(HtmlHistPath.c_str(), HistImg);

              drawIpoints(Image, NewEntry.GetDescriptor());
              cvSaveImage(HtmlImagePath.c_str(), Image, params);
              cvReleaseImage(&HistImg);
            }
            //=====================================================================================
          }
          cvReleaseImage(&Image);
        }
        else
        {
          cout << "Unable to open image" << ImagePath.c_str() << "\n";
        }
      }
    }
    DbFile.close();
  }
  else
  {
    cout << "Unable to open database setup file..."; 
  }

  if (trainClassifier)
  {
    // Iterate through descriptors and count them
    unsigned desCnt = 0;
    for (unsigned i = 0; i < mSurfEntries.size(); i++)
    {
      desCnt += mSurfEntries.at(i).GetDescriptorCount();
    }
    cout << "Descriptors = " << desCnt << "\n";

    // Create matrix of descriptors for K-means clustering
    // Each row is a descriptor entry
    Mat allDescriptors = Mat(desCnt, 64, CV_32FC1, 0.0f);

    unsigned idx = 0;
    for (unsigned i = 0; i < mSurfEntries.size(); i++)
    {
      const IpVec& ipvec = mSurfEntries.at(i).GetDescriptor();
      for (unsigned j = 0; j < ipvec.size(); j++)
      {
        const Ipoint ipnt = ipvec.at(j);

        // Fill in a single row with the current descriptor
        for (int k = 0; k < 64; k++)
        {
          allDescriptors.at<float>(idx,k) = ipnt.descriptor[k];
        }
        idx++;
      }
    }

    mClusterCount = 200;
    int attempts = 1;
    mpLabels = new Mat(desCnt, 1, CV_32S);
    mpCenters = new Mat(desCnt, mClusterCount, CV_32F);
    TermCriteria termcrit = TermCriteria(TermCriteria::MAX_ITER, 5000, 0.0f);

    // Create the dictionary
    kmeans(allDescriptors, mClusterCount, *mpLabels, termcrit, attempts, KMEANS_PP_CENTERS, mpCenters);

    idx = 0;
    for (unsigned i = 0; i < mSurfEntries.size(); i++)
    {
      CSurfEntry& Entry = mSurfEntries.at(i);
      for (unsigned j = 0; j < Entry.GetDescriptorCount(); j++)
      {
        Entry.SetClusterLabel(j, mpLabels->at<unsigned>(idx,0));
        idx++;
      }
    }

    const unsigned rowDim = mSurfEntries.size();
    const unsigned colDim = mClusterCount;

    Mat* mpWordHist  = new Mat(rowDim, colDim, CV_32F, Scalar(0));
    Mat* mpWordLabel = new Mat(rowDim, 1, CV_32F, Scalar(0));
    idx = 0;
    for (int i = 0; i < rowDim; i++)
    {
      CSurfEntry& Entry = mSurfEntries.at(i);
      for (unsigned j = 0; j < Entry.GetDescriptorCount(); j++)
      {
        unsigned clusterLabel = mpLabels->at<unsigned>(idx,0);
        mpWordHist->at<float>(i, clusterLabel)++;
        idx++;
      }
      mpWordLabel->at<float>(i,0) = (float)Entry.GetLabelId();
    }

    // Generate HTML each time
    if (mGenerateHtml)
    {
      for (unsigned i = 0; i < mSurfEntries.size(); i++)
      {
        CSurfEntry& entry = mSurfEntries.at(i);

        // Create html summary for the current entry
        string entryHtmlPath = mPaths.mHtmlPath + mName + "/" + entry.GetName() + ".html";
        WriteEntryHtmlSummary(entry, entryHtmlPath);

        // Create a summary for the library
        const unsigned histBins   = mClusterCount;
        const unsigned histEdge   = 15;
        const unsigned histHeight = 100;

        // Generate word histogram for the current image
        Mat wordHistImage = Mat(2*histEdge + histHeight, 2*histEdge + 3*histBins, CV_8UC3, Scalar(0));

        // Draw background and labels
        DrawHistBar(wordHistImage, histBins, histEdge, histHeight);

        const string wordHistImageName = entry.GetName() + ".wordhist.png";
        const string wordHistImagePath = mPaths.mHtmlPath + mName + "/" + wordHistImageName;

        Mat rowNormalized = Mat(mpWordHist->size(), CV_32F);
        cv::normalize(mpWordHist->row(i), rowNormalized, 0, histHeight, CV_MINMAX);
        DrawHistogram(rowNormalized, wordHistImage, Scalar(0xff, 0xff, 0xff, 0),
                      histBins, histEdge, histHeight);

        imwrite(wordHistImagePath.c_str(), wordHistImage);
      }
    }

    // SVM Classifier
    CvSVMParams svmParams = CvSVMParams();

    svmParams.svm_type = CvSVM::C_SVC;
    svmParams.kernel_type = CvSVM::RBF;
    svmParams.gamma = 0.1;
    svmParams.degree = 4;
    mpSVMClassifier = new CvSVM();
    mpSVMClassifier->train(*mpWordHist, *mpWordLabel, Mat(), Mat(), svmParams);

  }
}

//=================================================================================================
// Destructor
//=================================================================================================
CSurfDatabase::~CSurfDatabase()
{
}

//=================================================================================================
//=================================================================================================
void CSurfDatabase::SetSVMTrainAutoParams(CvParamGrid& c_grid,
                                          CvParamGrid& gamma_grid,
                                          CvParamGrid& p_grid,
                                          CvParamGrid& nu_grid,
                                          CvParamGrid& coef_grid,
                                          CvParamGrid& degree_grid)
{
    c_grid = CvSVM::get_default_grid(CvSVM::C);

    gamma_grid = CvSVM::get_default_grid(CvSVM::GAMMA);

    p_grid = CvSVM::get_default_grid(CvSVM::P);
    p_grid.step = 0;

    nu_grid = CvSVM::get_default_grid(CvSVM::NU);
    nu_grid.step = 0;

    coef_grid = CvSVM::get_default_grid(CvSVM::COEF);
    coef_grid.step = 0;

    degree_grid = CvSVM::get_default_grid(CvSVM::DEGREE);
    degree_grid.step = 0;
}

//=================================================================================================
//=================================================================================================
void CSurfDatabase::UpdateLabels(string ImageLabel)
{
  if (mLabels.size() == 0)
  {
    mLabels.push_back(ImageLabel);
  }
  else
  {
    if (!(mLabels.at(mLabels.size()-1) == ImageLabel))
    {
      mLabels.push_back(ImageLabel);
    }
  }
}

//=================================================================================================
//=================================================================================================
std::string CSurfDatabase::GetLabelName(unsigned LabelId)
{
  if (LabelId > (mLabels.size()-1))
  {
    return mLabels.at(0);
  }
  return mLabels.at(LabelId);
}

//=================================================================================================
//=================================================================================================
int CSurfDatabase::GetLabelId(string Label)
{
  for(unsigned i = 0; i < mLabels.size(); i++)
  {
    if (mLabels.at(i) == Label)
    {
      return i;
    }
  }
  return -1;
}

//=================================================================================================
//=================================================================================================
unsigned CSurfDatabase::ClassifyEntrySVM(CSurfEntry& Entry)
{

  Mat descrip = Mat(Entry.GetDescriptorCount(), 64, CV_32F, Scalar(0));

  for (unsigned i = 0; i < Entry.GetDescriptorCount(); i++)
  {
    const Ipoint& point = Entry.GetDescriptor().at(i);
    for (unsigned j = 0; j < 64; j++)
    {
      descrip.at<float>(i,j) = point.descriptor[j];
    }
  }

  CvMat* pEntryHist = cvCreateMat(1, mClusterCount, CV_32F);
  float* pEntryHistData = pEntryHist->data.fl;
  for (int i = 0; i < mClusterCount; i++)
  {
    pEntryHistData[i] = 0;
  }

  // Find the cluster label for each descripter (row)
  for (int i = 0; i < descrip.rows; i++)
  {
    double smallestNorm = DBL_MAX;
    unsigned clusterLabel = 0;

    for(int j = 0; j < mpCenters->rows; j++)
    {
      double curNorm = norm(descrip.row(i)- mpCenters->row(j));
      if (curNorm < smallestNorm)
      {
        smallestNorm = curNorm;
        clusterLabel = j;
      }
    }
    pEntryHistData[clusterLabel]++;
  }

  return (unsigned)mpSVMClassifier->predict(pEntryHist);
}

//=================================================================================================
// Description:
//  Takes a single SURF entry, compares its descriptors against every image in the database using
//  nearest neighbors search and then returns the label identifier that corresponds to the best
//  match. This version of the function is designed for speed and does not store any debug
//  information.
//=================================================================================================
bool CSurfDatabase::MatchEntryNN(CSurfEntry& Entry, unsigned& LabelId, unsigned& BestMatchCount)
{

  const unsigned minMatchCount = 3;
  if (Entry.GetDescriptorCount() < minMatchCount) return false;

  unsigned BestMatch = 0;
  unsigned MatchCount = 0;

  for (unsigned i = 0; i < mSurfEntries.size(); i++)
  {
    mSurfEntries.at(i).Compare(Entry);
    if (mSurfEntries.at(i).GetMatchCount() > MatchCount)
    {
      MatchCount = mSurfEntries.at(i).GetMatchCount();
      BestMatch = i;
    }
  }

  LabelId = mSurfEntries.at(BestMatch).GetLabelId();
  BestMatchCount = MatchCount;

  if (MatchCount < minMatchCount) return false;

  return true;
}

//=================================================================================================
// Description:
// Similar to the function above except html files/images are written in process to provide debug
// information
//=================================================================================================
unsigned CSurfDatabase::MatchEntry(CSurfEntry& Entry,
                                    string DbName,
                                    ofstream& Os,
                                    unsigned& BestMatchCount) // The debug version
{
  unsigned BestMatch = 0;
  unsigned MatchCount = 0;

  Os << "<table border=0 cellpadding=3 cellspacing=3>\n";

  for (unsigned i = 0; i < mSurfEntries.size(); i++)
  {
    Os << "<tr>\n";
    Os << "<td>\n";
    string DbHtmlPath    = "../" + mName + "/"  + mSurfEntries.at(i).GetName() + ".html";
    string EntryHtmlPath = "../" + DbName + "/" + Entry.GetName()              + ".html";

    Os << "<a href='" << DbHtmlPath << "'>" ;
    Os << mSurfEntries.at(i).GetName() << "</a>";
    Os << " vs. ";
    Os << "<a href='" << EntryHtmlPath << "'>";
    Os << Entry.GetName() << "</a>";
    Os << "<br>\n";

    mSurfEntries.at(i).Compare(Entry, Os);

    Os << "</td>\n";
    Os << "<td>\n";
    if (mSurfEntries.at(i).GetMatchCount())
    {
      string DbImagePath    = "html/" + mName + "/"  + mSurfEntries.at(i).GetName() + ".jpg";
      string EntryImagePath = "html/" + DbName + "/" + Entry.GetName()             + ".jpg";

      // Open the image
      IplImage *DbImage    = cvLoadImage(DbImagePath.c_str());
      IplImage *EntryImage = cvLoadImage(EntryImagePath.c_str());

      // If we opened a valid image
      if (DbImage && EntryImage)
      {

        /* get properties, needed to create dest image */
        int width  = DbImage->width + EntryImage->width;
        int height = 0;

        if (DbImage->height > EntryImage->height)
        {
          height = DbImage->height;
        }
        else
        {
          height = EntryImage->height;
        }

        int depth     = DbImage->depth;
        int nchannels = DbImage->nChannels;

        /* create destination image */
        IplImage *CombinedImage = cvCreateImage(cvSize(width, height), depth, nchannels);

        cvSetImageROI(CombinedImage, cvRect(0, 0, DbImage->width, DbImage->height));
        cvCopy(DbImage, CombinedImage, NULL);

        cvSetImageROI(CombinedImage, cvRect(DbImage->width, 0, EntryImage->width, EntryImage->height));
        cvCopy(EntryImage, CombinedImage, NULL);

        const IpVec& DbVec = mSurfEntries.at(i).GetDescriptor();
        const IpVec& EntryVec = Entry.GetDescriptor();

        string SavePath = "html/" + DbName + "Vs" + mName + "/" + Entry.GetName() + "vs" + mSurfEntries.at(i).GetName() + ".jpg";

        int p[3];
        p[0] = CV_IMWRITE_JPEG_QUALITY;
        p[1] = 100;
        p[2] = 0;

        cvSetImageROI(CombinedImage, cvRect(0, 0, width, height));

       for (unsigned j = 0; j < mSurfEntries.at(i).mMatchIndex.size(); j++)
       {
          int MatchIndex = mSurfEntries.at(i).mMatchIndex.at(j);
          if (MatchIndex != -1)
          {
            unsigned FromPixelX = (unsigned)DbVec.at(j).x;
            unsigned FromPixelY = (unsigned)DbVec.at(j).y;
            unsigned ToPixelX = (unsigned)EntryVec.at(MatchIndex).x + DbImage->width;
            unsigned ToPixelY = (unsigned)EntryVec.at(MatchIndex).y;
            cvLine(CombinedImage, cvPoint(FromPixelX, FromPixelY), cvPoint(ToPixelX, ToPixelY), CV_RGB(255,0,255));
          }
        }

        cvSaveImage(SavePath.c_str(), CombinedImage, p);

        Os << "<img src='" << Entry.GetName() << "vs" << mSurfEntries.at(i).GetName() << ".jpg'>\n";

        cvReleaseImage(&CombinedImage);
        cvReleaseImage(&DbImage);
        cvReleaseImage(&EntryImage);
      }

      Os << "</td>\n";
      Os << "</tr>\n";
    }

    if (mSurfEntries.at(i).GetMatchCount() > MatchCount)
    {
      MatchCount = mSurfEntries.at(i).GetMatchCount();
      BestMatch = i;
    }
  }

  BestMatchCount = MatchCount;
  return mSurfEntries.at(BestMatch).GetLabelId();
}

//=================================================================================================
//=================================================================================================
void CSurfDatabase::ClassifyDatabase(CSurfDatabase& Database)
{

  unsigned matchCountSVM = 0;
  unsigned matchCountBayes = 0;
  unsigned totalCount = 0;

  for (unsigned i = 0; i < Database.GetEntryCount(); i++)
  {
    unsigned labelSVM      = ClassifyEntrySVM(Database.GetSurfEntry(i));
    unsigned actualLabel   = Database.GetSurfEntry(i).GetLabelId();
    string labelNameSVM    = GetLabelName(labelSVM);
    string labelNameActual = Database.GetLabelName(actualLabel);

    if (labelNameSVM == labelNameActual)
    {
      matchCountSVM++;
    }

    totalCount++;
    
    cout << i << " SVM/Actual = " << "/" << labelNameSVM << "/" << labelNameActual << "\n";
  }

  cout << "SVM Result = " << 100.0f*(double)matchCountSVM/(double)totalCount << "\n";

}

//=================================================================================================
//=================================================================================================
void CSurfDatabase::MatchDatabase(CSurfDatabase& Database)
{
  string MatchDbSummaryPath;
  string MatchSummaryDir;
  ofstream MatchDbSummary;

  //This generates an executive summary of database matches
  if (mGenerateHtml)
  {
    MatchSummaryDir = Database.GetName() + "Vs" + GetName() + "/";

    MatchDbSummaryPath = mPaths.mHtmlPath + MatchSummaryDir + "Summary.html";
    MatchDbSummary.open(MatchDbSummaryPath.c_str());

    if (MatchDbSummary.is_open())
    {
      MatchDbSummary << "<html><title>" << mName << " vs. " << Database.GetName() << "</title>\n";
      MatchDbSummary << "<body>\n";
      MatchDbSummary << "<table border=0 cellpadding=2 cellspacing=2>\n";
      MatchDbSummary << "<tr>\n";
      MatchDbSummary << "<td><b>Summary</b></td>\n";
      MatchDbSummary << "<td><b>Truth</b></td>\n";
      MatchDbSummary << "<td><b>Match</b></td>\n";
      MatchDbSummary << "<td><b>Match Count</b></td>\n";
      MatchDbSummary << "</tr>\n";
    }
  }

  unsigned SuccessCount = 0;
  unsigned TotalCount = 0;
  // For each input entry figure out which one of "my" entries matches the best
  for(unsigned i = 0; i < Database.GetEntryCount(); i++)
  {
    // Get the input entry
    CSurfEntry Entry = Database.GetSurfEntry(i);

    unsigned MatchLabelId;

    if (mGenerateHtml)
    {
      string MatchEntrySummaryPath = mPaths.mHtmlPath + MatchSummaryDir + Entry.GetName() + ".html";
      ofstream MatchEntrySummary;

      MatchEntrySummary.open(MatchEntrySummaryPath.c_str());

      if (MatchEntrySummary.is_open())
      {
        unsigned BestMatchCount = 0;
        MatchEntrySummary << "<html><title>" << Entry.GetLabelId() << "</title>\n";
        MatchEntrySummary << "<body>\n";

        // This is where the current Surf entry (at the ith index) gets matched to the input database
        MatchLabelId = MatchEntry(Entry, Database.GetName(), MatchEntrySummary, BestMatchCount);

        MatchEntrySummary << "</body></html>\n";
        MatchEntrySummary.close();

        // Write the database summary entry
        if (MatchDbSummary.is_open())
        {
          MatchDbSummary << "<tr>\n";
          // Summary
          MatchDbSummary << "<td><a href='" << Entry.GetName() + ".html'>" << i << "</a></td>\n";
          // Truth
          MatchDbSummary << "<td><a href='../" << Database.GetName();
          MatchDbSummary << "/" << Entry.GetName() << ".html'>";
          MatchDbSummary << Database.GetLabelName(Entry.GetLabelId());
          MatchDbSummary << "</a></td>\n";
          // Match
          MatchDbSummary << "<td><a href='../" << GetName();
          MatchDbSummary << "/" << GetLabelName(MatchLabelId) << ".html'>";
          MatchDbSummary << GetLabelName(MatchLabelId) << "</a></td>\n";
          // Match count
          MatchDbSummary << "<td>" << BestMatchCount << " / ";
          MatchDbSummary << GetSurfEntry(MatchLabelId).GetDescriptorCount() << "</td>\n";
          MatchDbSummary << "</tr>\n";
        }

      if (GetLabelName(MatchLabelId) == Database.GetLabelName(Entry.GetLabelId()))
      {
        SuccessCount++;
      }
      TotalCount++;
      }
    }
    else
    {
      unsigned label = 0;
      unsigned matchCount = 0;
      // Do something else with the fast version
      MatchLabelId = Database.MatchEntryNN(Entry, label, matchCount);
    }
  }

  if (mGenerateHtml)
  {
    MatchDbSummary.precision(4);
    MatchDbSummary << "<b>Success Rate: </b> " << 100.0f*(float)SuccessCount/(float)TotalCount << "%\n";
    MatchDbSummary << "</table>\n";
    MatchDbSummary << "</body></html>\n";
    MatchDbSummary.close();
  }
}

//=================================================================================================
//=================================================================================================
CSurfEntry& CSurfDatabase::GetSurfEntry(unsigned i)
{
  return mSurfEntries.at(i);
}

//=================================================================================================
//=================================================================================================
unsigned CSurfDatabase::GetEntryCount() const
{
  return mSurfEntries.size();
}

//=================================================================================================
//=================================================================================================
string& CSurfDatabase::GetName()
{
  return mName;
}

//=================================================================================================
// Description: 
//  Reads a line from a database setup file and extracts the upright property
//=================================================================================================
bool CSurfDatabase::ReadUpright(ifstream& DbFile)
{
  string line;

  if (!DbFile.eof())
  {
    getline(DbFile, line);

    // Attempt to find a space and get its index
    unsigned spaceIdx = line.find_first_of(" ");

    // Make sure the space is less than the last index
    if (spaceIdx < (line.size()-2))
    {
      //Make sure the tag is Upright
      if (line.substr(0, spaceIdx) == "Upright")
      {
        // Check to see if the setting is set
        if (line.substr(spaceIdx+1, line.size()) == "true")
        {
          return true;
        }
      }
    }
  }

  // The default is to return false
  return false;
}

//=================================================================================================
// Description: 
//  Reads a line from a database setup file and extracts the number of octaves property
//=================================================================================================
unsigned CSurfDatabase::ReadOcatves(ifstream& DbFile)
{

  string line;

  if (!DbFile.eof())
  {
    getline(DbFile, line);

    // Attempt to find a space and get its index
    unsigned spaceIdx = line.find_first_of(" ");

    // Make sure the space is less than the last index
    if (spaceIdx <= (line.size()-2))
    {
      //Make sure the tag is Octaves
      if (line.substr(0, spaceIdx) == "Octaves")
      {
        string octavesStr = line.substr(spaceIdx+1, line.size());
        unsigned octaves = atoi(octavesStr.c_str());

        if (octaves <= 10)
        {
          return octaves;
        }
      }
    }
  }

  // The default is to return 4 octaves
  return 4;
}

//=================================================================================================
// Description: 
//  Reads a line from a database setup file and extracts the intervals property
//=================================================================================================
unsigned CSurfDatabase::ReadIntervals(ifstream& DbFile)
{

  string line;

  if (!DbFile.eof())
  {
    getline(DbFile, line);

    // Attempt to find a space and get its index
    unsigned spaceIdx = line.find_first_of(" ");

    // Make sure the space is less than the last index
    if (spaceIdx <= (line.size()-2))
    {
      //Make sure the tag is Intervals
      if (line.substr(0, spaceIdx) == "Intervals")
      {
        string intervalsStr = line.substr(spaceIdx+1, line.size());
        unsigned intervals = atoi(intervalsStr.c_str());

        if (intervals <= 10)
        {
          return intervals;
        }
      }
    }
  }

  // The default is to return 4 intervals
  return 4;
}

//=================================================================================================
// Description: 
//  Reads a line from a database setup file and extracts the initial samples property
//=================================================================================================
unsigned CSurfDatabase::ReadInitSample(ifstream& DbFile)
{
  string line;

  if (!DbFile.eof())
  {
    getline(DbFile, line);

    // Attempt to find a space and get its index
    unsigned spaceIdx = line.find_first_of(" ");

    // Make sure the space is less than the last index
    if (spaceIdx <= (line.size()-2))
    {
      //Make sure the tag is InitSample
      if (line.substr(0, spaceIdx) == "InitSample")
      {
        string initSampleStr = line.substr(spaceIdx+1, line.size());
        unsigned initSample = atoi(initSampleStr.c_str());

        if (initSample <= 4)
        {
          return initSample;
        }
      }
    }
  }

  // The default is to return 2 InitSamples
  return 2;
}

//=================================================================================================
// Description: 
//  Reads a line from a database setup file and extracts the threshold property
//=================================================================================================
float CSurfDatabase::ReadThreshold(ifstream& DbFile)
{
  string line;

  if (!DbFile.eof())
  {
    getline(DbFile, line);

    // Attempt to find a space and get its index
    unsigned spaceIdx = line.find_first_of(" ");

    // Make sure the space is less than the last index
    if (spaceIdx < (line.size()-2))
    {
      //Make sure the tag is Threshold
      if (line.substr(0, spaceIdx) == "Threshold")
      {
        string thresholdStr = line.substr(spaceIdx+1, line.size());
        float threshold = (float)atof(thresholdStr.c_str());

        if (threshold <= 0.03)
        {
          return threshold;
        }
      }
    }
  }

  // The default is to return a 0.003 threshold
  return 0.003f;
}

//=================================================================================================
// Description: 
//  Reads a line from a database setup file and extracts the auto levels property
//=================================================================================================
bool CSurfDatabase::ReadAutoLevels(ifstream& DbFile)
{
  string line;

  if (!DbFile.eof())
  {
    getline(DbFile, line);

    // Attempt to find a space and get its index
    unsigned spaceIdx = line.find_first_of(" ");

    // Make sure the space is less than the last index
    if (spaceIdx < (line.size()-2))
    {
      //Make sure the tag is AutoLevels
      if (line.substr(0, spaceIdx) == "AutoLevels")
      {
        // Check to see if the setting is true
        if (line.substr(spaceIdx+1, line.size()) == "true")
        {
          return true;
        }
      }
    }
  }

  // The default is to return false
  return false;
}

//=================================================================================================
// Description: 
//  Reads a line from a database setup file and extracts the generate html property
//=================================================================================================
bool CSurfDatabase::ReadGenerateHtml(ifstream& DbFile)
{
  string line;

  if (!DbFile.eof())
  {
    getline(DbFile, line);

    // Attempt to find a space and get its index
    unsigned spaceIdx = line.find_first_of(" ");

    // Make sure the space is less than the last index
    if (spaceIdx < (line.size()-2))
    {
      //Make sure the tag is GenerateHtml
      if (line.substr(0, spaceIdx) == "GenerateHtml")
      {
        // Check to see if the setting is true
        if (line.substr(spaceIdx+1, line.size()) == "true")
        {
          return true;
        }
      }
    }
  }

  // The default is to return false
  return false;
}

//=================================================================================================
// Description:
//  Entry is assumed to be part of the object's database (i.e. in this->mSurfEntries)
//=================================================================================================
void CSurfDatabase::WriteEntryHtmlSummary(CSurfEntry& Entry, std::string HtmlPath)
{

  // Construct the full path
  //string ImagePath = ImageDir + mName + "/" + Entry.GetName() + ".jpg";

  ofstream HtmlSummary(HtmlPath.c_str());

  if (HtmlSummary.is_open())
  {
    HtmlSummary << "<html>\n<title>" << Entry.GetName() << "</title>\n<body>\n";
    HtmlSummary << "<table cellpadding='4' cellspacing='4'><tr>\n";
    HtmlSummary << "<td><img src='../../images/" << mName << '/' << Entry.GetName() << ".jpg'></td>\n";
    HtmlSummary << "<td><img src='" << Entry.GetName() << ".jpg'></td>\n";
    HtmlSummary << "</tr><tr><td><b>Original</b></td>\n";
    HtmlSummary << "<td><b>Autoleveled with descriptors</b></td></tr>\n";
    HtmlSummary << "</table>\n<br>\n";
    HtmlSummary << "<img src='" << Entry.GetName() << ".hist.png'><br><br>\n";
    HtmlSummary << "<b>Database name: </b>" << mName << "<br>\n";
    HtmlSummary << "<b>Auto levels: </b>" << mAutoLevels << "<br>\n";
    HtmlSummary << "<b>SURF Parameters: </b>";
    HtmlSummary << "<ul><li><b>Upright: </b>"    << mParams.mUpright    << "<br>\n";
    HtmlSummary << "<li><b>Octaves: </b>"        << mParams.mOctaves    << "<br>\n";
    HtmlSummary << "<li><b>Intervals: </b>"      << mParams.mIntervals  << "<br>\n";
    HtmlSummary << "<li><b>Initial Sample: </b>" << mParams.mInitSample << "<br>\n";
    HtmlSummary << "<li><b>Threshold: </b>"      << mParams.mThreshold  << "</ul>\n";
    Entry.WriteHtmlInfo(HtmlSummary);
    HtmlSummary << "\n";
    HtmlSummary << "</body></html>\n";
    HtmlSummary.close();
  }
  else
  {
    cout << "Could not create/open " << HtmlPath << "\n";
  }
}
