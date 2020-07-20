#include "dataio.hpp"
#include "utility.hpp"
#include "map_viewer.h"

#define screen_width 1920
#define screen_height 1080

using namespace lo;
using namespace std;

int main(int argc, char **argv)
{
    std::string pc_folder = argv[1];
    std::string bbx_folder = argv[2];

    DataIo<Point_T> dataio;
    MapViewer<Point_T> mviewer(0.0); //downsampling ratio

    std::vector<std::string> pc_filenames;
    dataio.batch_read_filenames_in_folder(pc_folder, "_filelist.txt", ".txt", pc_filenames);
    std::vector<std::string> bbx_filenames;
    dataio.batch_read_filenames_in_folder(bbx_folder, "_filelist.txt", ".txt", bbx_filenames);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> bbx_viewer;
    bbx_viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("3D Viewer"));
    mviewer.set_interactive_events(bbx_viewer, screen_width, screen_height);

    int frame_num = bbx_filenames.size();

    pcTPtr current_pc_l0(new pcT());
    pcTPtr current_pc_l1(new pcT());
    pcTPtr bbxs_vertex(new pcT());
    std::vector<int> bbx_class;

    for (int i = 0; i < frame_num; i++) //two lidars for one frame
    {
        dataio.read_cloud_file(pc_filenames[2 * i], current_pc_l0);
        dataio.read_cloud_file(pc_filenames[2 * i + 1], current_pc_l1);
        dataio.read_bbx_8pts_with_type(bbx_filenames[i], bbxs_vertex, bbx_class);

        mviewer.display_pc_with_bbx_realtime(bbx_viewer, current_pc_l0, current_pc_l1, bbxs_vertex, bbx_class, 5);
        pcT().swap(*current_pc_l0);
        pcT().swap(*current_pc_l1);
        pcT().swap(*bbxs_vertex);
        std::vector<int>().swap(bbx_class);
    }

    return 1;
}
