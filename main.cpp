#include "../../../Qing/qing_common.h"
#include "../../../Qing/qing_dir.h"
#include "../../../Qing/qing_disp.h"
#include "../../../Qing/qing_io.h"
#include "../../../Qing/qing_ply.h"

string wnd_name = "mask";
Mat img;
vector<cv::Point> pts;

static void onMouse( int event, int x, int y, int, void* )
{
    if( EVENT_LBUTTONDOWN == event) {      //selected corners manual and refined by response
        Point seed = Point(x,y);
        cout << seed << " was selected" << endl;
        cv::circle(img, seed, 10, Scalar(255,0,0), 2);
        imshow(wnd_name, img);

        pts.push_back(seed);
    }
}

void get_masks_manual(string frm_dir, vector<string> files) {
    int size = files.size();

    string img_file, msk_file;
    Mat msk;
    Point2i poly_pts[1][4];
    const Point2i * ppt[1] = {poly_pts[0]};
    int npt[] = {4};

    for(int i = 0; i < size; ++i) {
        img_file = frm_dir + "/" + files[i];
        img = imread(img_file, 1);

        pts.clear();

        namedWindow( wnd_name, 0 );
        setMouseCallback( wnd_name, onMouse, 0);

        for(;;)
        {
            imshow(wnd_name, img);
            char c = (char)waitKey(0);
            if ( c == 10 && 4==pts.size()) {
                cout << "exit and generate mask...\n";
                break;
            }
        }
        destroyWindow(wnd_name);

        poly_pts[0][0] = pts[0];
        poly_pts[0][1] = pts[1];
        poly_pts[0][2] = pts[2];
        poly_pts[0][3] = pts[3];

        msk = Mat::zeros(img.size(), CV_8UC1);
        for(int j = 0; j < pts.size(); ++j) {
            cout << pts[j] << endl;
        }
        cv::fillPoly(msk, ppt, npt, 1,  Scalar(255));
        cout << "HHH" << endl;
        msk_file = frm_dir + "msk_" + files[i];
        imwrite(msk_file, msk);
        cout << "saving " << msk_file << endl;

    }

}

int main(int argc, char * argv[])
{
    cout << "Usage: " << argv[0] << endl;
    string img_folder = "/media/ranqing/ranqing_wd/ZJU/HumanDatas/20161224/Calib_rectified";
    string cal_folder = "/media/ranqing/ranqing_wd/ZJU/HumanDatas/20161224/Calib_results";
    string wnd_name;


    int num = 60;
    string camnames[60] = {"A01", "A02", "A03", "A04", "A05", "A06", "A07", "A08", "A09", "A10", "A11", "A12", "A13", "A14", "A15", "A16",
                           "B01", "B02", "B03", "B04", "B05", "B06", "B07", "B08", "B09", "B10", "B11", "B12", "B13", "B14", "B15", "B16",
                           "C01", "C02", "C03", "C04", "C05", "C06", "C07", "C08", "C09", "C10", "C11", "C12", "C13", "C14", "C15", "C16",
                           "L01", "L02", "L03", "L04", "L05", "L06", "R01", "R02", "R03", "R04", "R05", "R06"};

    string result_folder = "../depths/";
    qing_create_dir(result_folder);
    cout << "result_folder_name: " << result_folder << endl;

    //    for(int i = 0; i < num; i+=2) {
    //        string frm_0 = img_folder + "/" + camnames[i] + "/";
    //        string frm_1 = img_folder + "/" + camnames[i+1] + "/";
    //        vector<string> img_files_0(0);
    //        vector<string> img_files_1(0);
    //        qing_get_all_files(frm_0, img_files_0);
    //        qing_get_all_files(frm_1, img_files_1);

    //        // get_masks_manual(frm_0, img_files_0);
    //        get_masks_manual(frm_1, img_files_1);

    //    }

    Size board_size(14, 24);
    string frm_0, frm_1, img_name_0, img_name_1, msk_name_0, msk_name_1, stereo_file, disp_name, ply_name;
    vector<string> img_files_0(0), img_files_1(0);
    vector<Point2f> corners_0(0), corners_1(0);
    Mat img0, img1, gray_img0, gray_img1, msk0, msk1, raw_disp, disp, show_disp;
    Mat qmatrix(4, 4, CV_64F);
    bool found0 , found1;
    float coff[3] = {0.0f}, offset;

    vector<Vec3f> points(0);
    vector<Vec3f> colors(0);

    for(int i = 0; i < num; ++i) {
        frm_0 = img_folder + "/" + camnames[i] + "/";
        frm_1 = img_folder + "/" + camnames[i+1] + "/";
        stereo_file = cal_folder + "/stereo_" + camnames[i] + camnames[i+1] + ".yml";
        cout << stereo_file << endl;
        qing_read_stereo_yml_qmatrix(stereo_file, qmatrix);
        cout << "qmatrix: " << endl << qmatrix << endl;

        result_folder = result_folder +  camnames[i] + camnames[i+1];
        qing_create_dir(result_folder);

        img_files_0.clear();
        img_files_1.clear();

        qing_get_all_files(frm_0, img_files_0);
        qing_get_all_files(frm_1, img_files_1);
        sort(img_files_0.begin(), img_files_0.end());
        sort(img_files_1.begin(), img_files_1.end());

        //        for(int j = 0; j < img_files_0.size(); ++j) {
        //            cout << img_files_0[j] << endl;
        //            cout << img_files_1[j] << endl;
        //        }

        int files_size = img_files_0.size() * 0.5;

        for(int j = 0; j < files_size; ++j)
        {
            img_name_0 = frm_0 + img_files_0[j] ;
            msk_name_0 = frm_0 + img_files_0[j+files_size];
            img_name_1 = frm_1 + img_files_1[j];
            msk_name_1 = frm_1 + img_files_1[j+files_size];

            cout << img_name_0 << '\n' << msk_name_0 << endl;
            cout << img_name_1 << '\n' << msk_name_1 << endl;


            img0 = imread(img_name_0, 1);
            img1 = imread(img_name_1, 1);
            msk0 = imread(msk_name_0, 0);   threshold(msk0, msk0, 125, 255, THRESH_BINARY);
            msk1 = imread(msk_name_1, 0);   threshold(msk1, msk1, 125, 255, THRESH_BINARY);

            cvtColor(img0, gray_img0, CV_BGR2GRAY);
            cvtColor(img1, gray_img1, CV_BGR2GRAY);

            corners_0.clear();
            corners_1.clear();

            found0 = findChessboardCorners(gray_img0, board_size, corners_0);
            found1 = findChessboardCorners(gray_img1, board_size, corners_1);

            if(found0 && found1) {
                cornerSubPix(gray_img0, corners_0, Size(11,11), Size(-1,-1), TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20, 0.1));
                cornerSubPix(gray_img1, corners_1, Size(11,11), Size(-1,-1), TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20, 0.1));
            }

            qing_get_slanted_disparity_w(corners_0, corners_1, coff);
           // offset = ((int)(coff[2])/10)*10;
            cout << "slanted plane: " << coff[0] << '\t' << coff[1] << '\t' << coff[2] /*<< '\t' << offset*/ << endl;

            raw_disp = Mat::zeros(img0.size(), CV_32FC1);
            disp = Mat::zeros(img0.size(), CV_32FC1);
            show_disp = Mat::zeros(img0.size(), CV_8UC1);

            float * ptr_raw_disp = raw_disp.ptr<float>(0);
            float * ptr_disp = disp.ptr<float>(0);
            uchar * ptr_mask = msk0.ptr<uchar>(0);
            int idx = 0, h = img0.size().height, w = img0.size().width;
            float max_val = 0.f, min_val = 1000000.f;

            for(int y = 0; y < h; ++y) {
                for(int x = 0; x < w; ++x, ++idx) {

                    if(ptr_mask[idx] == 0) continue;
                    ptr_raw_disp[idx] = (float)qing_get_interpolate_disp_value(x, y, coff);
                    //ptr_disp[idx] = ptr_raw_disp[idx] - offset;
                    if(ptr_raw_disp[idx]< min_val)
                        min_val = ptr_raw_disp[idx];
                }
            }
            offset = min_val - 10;   cout << "disp offset = " << offset << endl;
            for(int k = 0; k < idx; ++k) {
                if(ptr_mask[idx] == 0) continue;
                ptr_disp[k] = ptr_raw_disp[k] - offset;
                if(max_val < ptr_disp[k])
                    max_val = ptr_disp[k];
            }
            cout << "max disp = " << max_val << endl;

            disp.convertTo(show_disp, CV_8UC1, 200/max_val);
//            imshow("disp", show_disp);
//            waitKey(0);
//            destroyWindow("disp");
            disp_name = result_folder + "/disp_" + img_files_0[j];
            imwrite(disp_name, show_disp);
            cout << "save " << disp_name << endl;

            points.clear();
            colors.clear();

            qing_disp_2_depth(points, colors, ptr_raw_disp, ptr_mask, img0.ptr<unsigned char>(0), qmatrix.ptr<double>(0), Point2i(0,0), img0.size().width, img0.size().height );
            ply_name = result_folder + "/point_" + img_files_0[j].substr(0, 4) + ".ply";
            qing_write_point_color_ply(ply_name, points, colors);
            cout << "save " << ply_name << endl;
          //  break;
        }

        break;
    }


    return 1;
}

