/*!
 \file dualfisheye2equi.cpp
 \brief Dualfisheye to equirectangular mapping (RGB image format), exploiting PeR core and io modules
 * example of command line :

  ./dualfisheye2equi ../data/RicohThetaS_calib_withTrans.xml ../media/ 0 9 1 ../data/maskFull.png ../media/poses.txt 1

 \param xmlFic the dualfisheye camera calibration xml file (the data directory stores an example of a dualfisheye camera xml file)
 \param imagesDir directory where dualfisheye images to read (with 6 digits before the extension) are and where the output equirectangular images will be written (with characters 'e_' before the 6 digits)
 \param iFirst the number of the first image to transform
 \param iLast the number of the last image to transform
 \param iStep the increment to the next image to transform
 \param maskFic the dualfisheye image mask to discard useless areas (png file: a black pixel is not to be transformed)
 \param posesFic a text file of one 3D pose per image (one row - one image) stored as the 3 elements of the translation vector followed by the 3 elements of the axis-angle vector representation of the rotation
 \param iInvertPose a flag to let the program knows if poses of posesFic must be applied to images as they are or inversed

 *
 \author Guillaume CARON
 \version 0.1
 \date August 2017
*/

#include <iostream>
#include <iomanip>

#include <per/prcommon.h>
#include <per/prOmni.h>
#include <per/prEquirectangular.h>
#include <per/prStereoModel.h>
#include <per/prStereoModelXML.h>

#include <boost/regex.hpp>
#include <boost/filesystem.hpp>

// VISP includes
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>

#include <visp/vpTime.h>

#include <visp/vpDisplayX.h>

#define INTERPTYPE prInterpType::IMAGEPLANE_BILINEAR

//#define VERBOSE

/*!
 * \fn main()
 * \brief Main function of the dual fisheye to equirangular spherical image transformation
 *
 * 1. Loading a divergent stereovision system made of two fisheye cameras considering the Barreto's model from an XML file got from the MV calibration software
 *
 *
 */
int main(int argc, char **argv)
{
    //Loading the twinfisheye intrinsic parameters
    if(argc < 2)
    {
#ifdef VERBOSE
        std::cout << "no XML stereo rig file given" << std::endl;
#endif
        return -1;
    }
    
    //Create an empty rig
    prStereoModel stereoCam(2);

    // Load the stereo rig parameters from the XML file
    {
        prStereoModelXML fromFile(argv[1]);
        
        fromFile >> stereoCam;
    }
    
    vpHomogeneousMatrix c2Rc1 = stereoCam.sjMr[1];

    c2Rc1[0][3] *= 0.5;
    c2Rc1[1][3] *= 0.5;
    c2Rc1[2][3] *= 0.5;
    
    vpHomogeneousMatrix s1Mr;
    s1Mr[0][3] = -c2Rc1[0][3];
    s1Mr[1][3] = -c2Rc1[1][3];
    s1Mr[2][3] = -c2Rc1[2][3];

#ifdef VERBOSE
    std::cout << "Loading the XML file to an empty rig..." << std::endl;
    
    // If a sensor is loaded, print its parameters
    if(stereoCam.get_nbsens() >= 1)
    {
        std::cout << "the stereo rig is made of a " << ((prCameraModel *)stereoCam.sen[0])->getName() << " camera of intrinsic parameters alpha_u = " << ((prCameraModel *)stereoCam.sen[0])->getau() << " ; alpha_v = " << ((prCameraModel *)stereoCam.sen[0])->getav() << " ; u_0 = " << ((prCameraModel *)stereoCam.sen[0])->getu0() << " ; v_0 = " << ((prCameraModel *)stereoCam.sen[0])->getv0();
        if(((prCameraModel *)stereoCam.sen[0])->getType() == Omni)
            std::cout << " ; xi = " << ((prOmni *)stereoCam.sen[0])->getXi();
        std::cout << "..." << std::endl;
    }

    // If a second sensor is loaded, print its parameters and its pose relatively to the first camera
    if(stereoCam.get_nbsens() >= 2)
    {
        std::cout << "... and a " << ((prCameraModel *)stereoCam.sen[1])->getName() << " camera of intrinsic parameters alpha_u = " << ((prCameraModel *)stereoCam.sen[1])->getau() << " ; alpha_v = " << ((prCameraModel *)stereoCam.sen[1])->getav() << " ; u_0 = " << ((prCameraModel *)stereoCam.sen[1])->getu0() << " ; v_0 = " << ((prCameraModel *)stereoCam.sen[1])->getv0();
        if(((prCameraModel *)stereoCam.sen[1])->getType() == Omni)
            std::cout << " ; xi = " << ((prOmni *)stereoCam.sen[1])->getXi();
        std::cout << "..." << std::endl;
   
        std::cout << "...at camera pose cpMco = " << std::endl << stereoCam.sjMr[1] << std::endl << " relative to the first camera." << std::endl;
    }
#endif
    
    //Loading the reference image with respect to which the cost function will be computed
    if(argc < 3)
    {
#ifdef VERBOSE
        std::cout << "no image files directory path given" << std::endl;
#endif
        return -4;
    }

    //Get filename thanks to boost
    char myFilter[1024];
    char *chemin = (char *)argv[2];
    char ext[] = "png";
    
    if(argc < 4)
    {
#ifdef VERBOSE
        std::cout << "no initial image file number given" << std::endl;
#endif
        return -6;
    }
    unsigned int i0 = atoi(argv[3]);//1;//0;
    
    if(argc < 5)
    {
#ifdef VERBOSE
        std::cout << "no image files count given" << std::endl;
#endif
        return -7;
    }
    unsigned int i360 = atoi(argv[4]);
    
    if(argc < 6)
    {
#ifdef VERBOSE
        std::cout << "no image sequence step given" << std::endl;
#endif
        return -8;
    }
    unsigned int iStep = atoi(argv[5]);
    
    //lecture de l'image "masque"
    //Chargement du masque
    vpImage<unsigned char> Mask;
    if(argc < 7)
    {
#ifdef VERBOSE
        std::cout << "no mask image given" << std::endl;
#endif
    }
    else
    {
        try
        {
            vpImageIo::read(Mask, argv[6]);
        }
        catch(vpException e)
        {
            std::cout << "unable to load mask file" << std::endl;
            return -8;
        }
    }
    
    //fichier avec les poses initiales r_0
    bool ficInit = false;
    std::vector<vpPoseVector> v_pv_init;
    if(argc < 8)
    {
#ifdef VERBOSE
        std::cout << "no initial poses file given" << std::endl;
#endif
        //return -9;
    }
    else
    {
        ficInit = true;
        
        std::ifstream ficPosesInit(argv[7]);
        if(!ficPosesInit.good())
        {
#ifdef VERBOSE
            std::cout << "poses file " << argv[7] << " not existing" << std::endl;
#endif
            return -9;
        }
        vpPoseVector r;
        while(!ficPosesInit.eof())
        {
            ficPosesInit >> r[0] >> r[1] >> r[2] >> r[3] >> r[4] >> r[5];
						r[0] = r[1] = r[2] = 0;
						//r[3] -= M_PI;
						//r[5] = r[5];
						//std::cout << r.t() << std::endl;
            v_pv_init.push_back(r);
        }
        ficPosesInit.close();
    }
    
    //direct or inverse pose
    unsigned int inversePose = 0;
    if(argc < 9)
    {
#ifdef VERBOSE
        std::cout << "no stabilisation parameter given" << std::endl;
#endif
        //return -9;
    }
    else
        inversePose = atoi(argv[8]);
    

    vpDisplayX disp;
    vpDisplayX disp2;
    
    
    //Pour chaque image du dataset
    int nbPass = 0;
    bool clickOut = false;
    unsigned int imNum = i0;
    double temps;
    std::vector<double> v_temps;
    v_temps.reserve((i360-i0)/iStep);
    
    vpHomogeneousMatrix cMc0, erMdf;
    erMdf.buildFrom(0, 0, 0, 0, 0, 0);
    //erMdf.buildFrom(0, 0, 0, 0, 0, -M_PI*0.5);
    
    vpImage<vpRGBa> I_df; //Dual fisheye image
    vpImage<vpRGBa> I_er; //Equirectangular image
    
    boost::filesystem::path dir(chemin);
    boost::regex my_filter;
    std::string name;
    std::ostringstream s;
    std::string filename;
    
    prEquirectangular equirectCam;
    prPointFeature P;
    double Xs, Ys, Zs, u, v, dv, du, unmdv, unmdu;
    unsigned int icam, imWidth, imHeight;
    unsigned int i, j;
    vpRGBa curPix, *pt_bitmap_er;
    while(!clickOut && (imNum <= i360))
    {
        temps = vpTime::measureTimeMs();
        std::cout << "num request image : " << nbPass << std::endl;
        
        //load source image
        sprintf(myFilter, "%06d.*\\.%s", imNum, ext);
        
        my_filter.set_expression(myFilter);
        
        for (boost::filesystem::directory_iterator iter(dir),end; iter!=end; ++iter)
        {
            name = iter->path().leaf().string();
            if (boost::regex_match(name, my_filter))
            {
                std::cout << iter->path().string() << " loaded" << std::endl;
                vpImageIo::read(I_df, iter->path().string());
              	filename = iter->path().filename().string();
                break;
            }
        }
        if(nbPass == 0)
        {
            imWidth = I_df.getWidth();
            imHeight = I_df.getHeight();
            I_er.resize(imHeight, imWidth);
            equirectCam.init(imWidth*0.5/M_PI, imHeight*0.5/(M_PI*0.5), imWidth*0.5, imHeight*0.5);
            
            disp.init(I_df, 50, 50, "Dual fisheye");
            disp2.init(I_er, 500, 50, "Equirectangular");
            
            if(Mask.getWidth() == 0)
                Mask.resize(I_df.getHeight(), I_df.getWidth(), 255);
        }

        vpDisplay::display(I_df);
        vpDisplay::flush(I_df);
        
        if(v_pv_init.size() > imNum)
        {
            cMc0.buildFrom(v_pv_init[imNum]);
            if(inversePose)
                cMc0 = cMc0.inverse();
        }
        else
            cMc0.eye();
        
        cMc0 = cMc0*erMdf;
        
        //Dual fisheye to equirectangular
        pt_bitmap_er = I_er.bitmap;
        double rho = 1.0; //5.0
        for(unsigned int v_er = 0 ; v_er < imHeight ; v_er++)
        {
            for(unsigned int u_er = 0 ; u_er < imWidth ; u_er++, pt_bitmap_er++)
            {
                //Equirectangular to sphere
                P.set_u(u_er);
                P.set_v(v_er);
                equirectCam.pixelMeterConversion(P);
                equirectCam.projectImageSphere(P, Xs, Ys, Zs);

                Xs *= rho;
                Ys *= rho;
                Zs *= rho;

                //sphere rotation
                P.set_oX(Xs);
                P.set_oY(Ys);
                P.set_oZ(Zs);
                P.changeFrame(cMc0);

                //sphere to dual fisheye
                if(P.get_Z() > 0)
                {
                    icam = 0;

                    P.sX = P.sX.changeFrame(s1Mr);
                }
                else
                {
                    icam = 1;
                    P.sX = P.sX.changeFrame(c2Rc1);
                }
                
                //if(P.get_Z() > 0.0)
                {
                    ((prOmni *)(stereoCam.sen[icam]))->project3DImage(P);
                    
                    ((prOmni *)(stereoCam.sen[icam]))->meterPixelConversion(P);
                    
                    u = P.get_u();
                    v = P.get_v();
                    
                    if( (u >= 0) && (v >= 0) && (u < (imWidth-1)) && (v < (imHeight-1))  )
                    {
                        switch(INTERPTYPE)
                        {
                            case IMAGEPLANE_BILINEAR:
                                curPix.R = curPix.G = curPix.B = 0;
                                
                                i = (int)v; dv = v-i; unmdv = 1.0-dv;
                                j = (int)u; du = u-j; unmdu = 1.0-du;
                                
                                
                                if (Mask[i][j] != 0)
                                {
                                    curPix = curPix + I_df[i][j]*unmdv*unmdu;
                                }
                                
                                if (Mask[i+1][j] != 0)
                                {
                                    curPix = curPix + I_df[i+1][j]*dv*unmdu;
                                }
                                
                                if (Mask[i][j+1] != 0)
                                {
                                    curPix = curPix + I_df[i][j+1]*unmdv*du;
                                }
                                
                                if (Mask[i+1][j+1] != 0)
                                {
                                    curPix = curPix + I_df[i+1][j+1]*dv*du;
                                }
                                
                                *pt_bitmap_er = curPix;
                                
                                break;
                            case IMAGEPLANE_NEARESTNEIGH:
                            default:
                                
                                i = vpMath::round(v);
                                j = vpMath::round(u);
                                
                                if(Mask[i][j] != 0)
                                {
                                    *pt_bitmap_er = I_df[i][j];
                                }
                                
                                break;
                        }
                    }
                }
            }
            //std::cout << P.get_u() << "\t" << P.get_v() << "\t" << P.get_x() << "\t" << P.get_y() << std::endl;
        }

        
        v_temps.push_back(vpTime::measureTimeMs()-temps);
        std::cout << "Pass " << nbPass << " time : " << v_temps[nbPass] << " ms" << std::endl;
        
        vpDisplay::display(I_er);
        vpDisplay::flush(I_er);
        
        clickOut=vpDisplay::getClick(I_er,false);
        

        //save the equirectangular image
        s.str("");
        s.setf(std::ios::right, std::ios::adjustfield);
        //s << chemin << "/e_" << std::setfill('0') << std::setw(6) << imNum << "." << ext;
        s << chemin << "/images_equirectangular/" << filename;
        filename = s.str();
        vpImageIo::write(I_er, filename);
        
        imNum+=iStep;
        nbPass++;
        //angle += 2.5*M_PI/180.;
    }
    
    //save times list to file
    s.str("");
    s.setf(std::ios::right, std::ios::adjustfield);
    s << chemin << "/e_time_" << i0 << "_" << i360 << ".txt";
    filename = s.str();
    std::ofstream ficTime(filename.c_str());
    std::vector<double>::iterator it_time = v_temps.begin();
    for(;it_time != v_temps.end() ; it_time++)
    {
        ficTime << *it_time << std::endl;
    }
    ficTime.close();
    
	return 0;
}
