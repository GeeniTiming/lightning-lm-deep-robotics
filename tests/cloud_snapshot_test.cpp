#include "common/cloud_snapshot.h"
#include <pcl/conversions.h>
#include <atomic>
#include <thread>
#include <iostream>
#include <stdexcept>
int main() {
 lightning::CloudSnapshot cache;
 if(cache.Copy())return 1;
 std::atomic<bool> done{false};std::atomic<unsigned> reads{0};
 std::thread writer([&]{
  lightning::PointCloudType cloud;
  for(int frame=1;frame<=30000;frame++) {
   cloud.resize(1+(frame%1000));
   for(auto &p:cloud.points){p.x=frame;p.y=frame;p.z=frame;}
   cache.Publish(cloud);
  }
  done=true;
 });
 auto reader=[&]{
  do {
   auto c=cache.Copy();if(!c)continue;
   if(c->points.size()!=size_t(c->width)*c->height)std::abort();
   float x=c->points.front().x;for(auto &p:c->points)if(p.x!=x || p.y!=x || p.z!=x)std::abort();
   pcl::PCLPointCloud2 out;pcl::toPCLPointCloud2(*c,out);++reads;
   c->clear(); // reader changes cannot corrupt later snapshots
  } while(!done);
 };
 std::thread a(reader),b(reader);writer.join();a.join();b.join();
 auto c=cache.Copy();if(!c || c->empty() || reads==0)return 2;
 std::cout<<"PASS 30000 frames, "<<reads<<" concurrent snapshot conversions; dimensions and point contents consistent"<<std::endl;
}
