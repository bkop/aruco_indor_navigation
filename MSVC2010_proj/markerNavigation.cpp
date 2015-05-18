/*****************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************/
 

/*
сделать : откалибровать камеру 
передать строки в поток 
отрисовать данные из потока в матлабе.
*/

#include <iostream>
#include "src/aruco.h"
#include "src/cvdrawingutils.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
using namespace cv;
using namespace aruco;
int main(int argc,char **argv)
{
    try
    {
        if (argc<2) {
            cerr<<"Usage: (in.jpg|in.avi) [cameraParams.yml] [markerSize] [outImage]"<<endl;
            exit(0);
        }


        aruco::CameraParameters CamParam;
        MarkerDetector MDetector;
        vector<Marker> Markers;
		vector<cv::Mat> Tmat_coll;
		float Tmat_temp[4][4];
		float Rmat_temp[3][3];
		//cv::Mat Rmat_temp(3,3,CV_32S);
		cv::Mat Tmat_basis(4,4,CV_32S);//4x4
		
        float MarkerSize=-1;
        //read the input image
        cv::Mat InImage;
        //try opening first as video
        VideoCapture vreader(argv[1]);
        if (vreader.isOpened()) {
            vreader.grab();
            vreader.retrieve(InImage);
        }
        else {
            InImage=cv::imread(argv[1]);
        }
        //at this point, we should have the image in InImage
        //if empty, exit
        if (InImage.total()==0) {
            cerr<<"Could not open input"<<endl;
            return 0;
        }

	//read camera parameters if specifed
        if (argc>=3) {
            CamParam.readFromXMLFile(argv[2]);
            //resizes the parameters to fit the size of the input image
            CamParam.resize( InImage.size());
        }
        //read marker size if specified
        if (argc>=4) MarkerSize=atof(argv[3]);
        cv::namedWindow("in",1);

	
	//Ok, let's detect
        MDetector.detect(InImage,Markers,CamParam,MarkerSize);
        //for each marker, draw info and its boundaries in the image

		//get transform matrices
		cout<<endl;
		cout<<"count transformed data"<<endl;
		for (unsigned int i=0;i<Markers.size();i++) {
            //cout<<Markers[i]<<endl;
			cout<<" count transformed data "<<i<<endl;
            //Markers[i].calculateExtrinsics(MarkerSize, CamParam, true);
			cv::Mat Rtemp=Mat(3,3,CV_32FC1,&Rmat_temp);
			cout<<"count rodrigues "<<i<<endl;
			Rodrigues(Markers[i].Rvec,Rtemp);
			//creating  transform matrix
			//T=[ R00 R01 R01 T0;R10 R11 R11 T1;R20 R21 R21 T2; 0 0 0 1]!!!
			cout<<"creating Transform matrix "<<i<<endl;
			Tmat_temp[0][0]=Rmat_temp[0][0]; Tmat_temp[0][1]=Rmat_temp[0][1]; Tmat_temp[0][2]=Rmat_temp[0][2];
			Tmat_temp[0][3]=Markers[i].Tvec.at<float>(0);
			Tmat_temp[1][0]=Rmat_temp[1][0]; Tmat_temp[1][1]=Rmat_temp[1][1]; Tmat_temp[1][2]=Rmat_temp[1][2];
			Tmat_temp[1][3]=Markers[i].Tvec.at<float>(1);			
			Tmat_temp[2][0]=Rmat_temp[2][0]; Tmat_temp[2][1]=Rmat_temp[2][1]; Tmat_temp[2][2]=Rmat_temp[2][2];
			Tmat_temp[2][3]=Markers[i].Tvec.at<float>(2);			
			Tmat_temp[3][0]=float(0); Tmat_temp[3][1]=float(0); Tmat_temp[3][2]=float(0);
			Tmat_temp[3][3]=1;
			cout<<"creating tempA matrix "<<i<<endl;
			//A = Mat(2, 5, CV_32FC1, &data);
			cv::Mat A=Mat(4,4,CV_32F,Tmat_temp);
			cout << "tmatrix A = "<< endl << " "  << A << endl << endl;
			cout<<"cloning tempA to Tmat_coll matrix "<<i<<endl<<endl<<endl;
			Tmat_coll.push_back(A.clone());
			//getting inverse basis matrix
			
			if (Markers[i].id==101){
				cout<<"inverse basis matrix "<<i<<endl;
				Tmat_basis=Tmat_coll[i].inv();
			} 
		}	
		//count new transform matrices for specified coordinate system
		cout<<"count new transform matrices "<<endl;
        for (unsigned int i=0;i<Markers.size();i++) {
		if (Markers[i].id!=101){
				//cv::Mat A=Mat(4,4,CV_32FC1,&Tmat_temp);
			cout<<"count new transform matrice "<<i<<endl;	
			Tmat_coll[i]=(Tmat_basis*Tmat_coll[i]);
			}
		cout<<"transformed data"<<Markers[i].id<<endl;
		cout << "M = "<< endl << " "  << Tmat_coll[i] << endl << endl;
		}

        for (unsigned int i=0;i<Markers.size();i++) {
            //cout<<Markers[i]<<endl;
            Markers[i].draw(InImage,Scalar(0,0,255),2);
        }
        //draw a 3d cube in each marker if there is 3d info
        if (  CamParam.isValid() && MarkerSize!=-1)
            for (unsigned int i=0;i<Markers.size();i++) {
                CvDrawingUtils::draw3dCube(InImage,Markers[i],CamParam);
            }
		
        //show input with augmented information
        cv::imshow("in",InImage);
	//show also the internal image resulting from the threshold operation
        cv::imshow("thes", MDetector.getThresholdedImage() );
        cv::waitKey(0);//wait for key to be pressed


        if (argc>=5) cv::imwrite(argv[4],InImage);
    } catch (std::exception &ex)

    {
        cout<<"Exception :"<<ex.what()<<endl;
    }
}
