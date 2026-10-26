#include <rog_map/prob_map.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <random>
#include <cstring>
#include <limits>
#include <stdexcept>

using namespace rog_map;
void require(bool ok, const char *message) { if (!ok) throw std::runtime_error(message); }
class BenchMap : public ProbMap {
public:
    BenchMap(const std::string &path, bool optimized, bool small, int batch=1, bool rays=true) {
        cfg_=Config(path); cfg_.raycast_optimized_enable=optimized;
        cfg_.bdm_search_enable=true; cfg_.corridor_obstacle_backend="bdm";
        if (small) {
            cfg_.map_size_d=Vec3f(12,12,6); cfg_.map_sliding_en=true;
            cfg_.batch_update_size=batch; cfg_.raycasting_en=rays;
            cfg_.resetMapSize();
        }
        initProbMap(); setStateChangeTrackingEnabled(true);
    }
    void rollover() { raycast_data_.observation_epoch=std::numeric_limits<uint32_t>::max()-1; }
    size_t stampBytes() const { return raycast_data_.observation_stamp.size()*sizeof(uint32_t); }
    const std::vector<double> &timings() const { return time_consuming_; }
    void compare(BenchMap &other, std::mt19937 &rng) {
        require(occupancy_buffer_==other.occupancy_buffer_,"probability mismatch");
        require(raycast_data_.hit_cnt==other.raycast_data_.hit_cnt,"hit count mismatch");
        require(raycast_data_.operation_cnt==other.raycast_data_.operation_cnt,"evidence count mismatch");
        auto a=raycast_data_.update_cache_id_g,b=other.raycast_data_.update_cache_id_g;
        require(a.size()==b.size(),"pending cache size mismatch");
        while (!a.empty()) { require(a.front()==b.front(),"pending cache order mismatch"); a.pop();b.pop(); }
        auto da=drainStateChanges(),db=other.drainStateChanges();
        require(da.size()==db.size(),"BDM state delta size mismatch");
        for(size_t i=0;i<da.size();++i)
            require(da[i].id_g==db[i].id_g && da[i].from_type==db[i].from_type && da[i].to_type==db[i].to_type,"BDM state delta mismatch");
        std::uniform_real_distribution<double> xy(-5,5),z(-1,4);
        for(int n=0;n<250;++n) {
            Vec3f pos=getLocalMapOrigin()+Vec3f(xy(rng),xy(rng),z(rng));
            require(getGridType(pos)==other.getGridType(pos),"classification mismatch");
            for(bool tracking:{false,true}) {
                auto ia=inflationMap(tracking),ib=other.inflationMap(tracking);
                require(ia->getGridType(pos)==ib->getGridType(pos),"inflation mismatch");
                if(n<30) {
                    Vec3i id;ia->infMapPosToGlobalIndex(pos,id);
                    std::vector<std::pair<int,int>> ra,rb;
                    require(ia->bdmIntervals(id.x(),id.y(),id.z()-30,id.z()+30,false,ra)==ib->bdmIntervals(id.x(),id.y(),id.z()-30,id.z()+30,false,rb) && ra==rb,"BDM interval mismatch");
                }
            }
            if(n<10) {
                vec_E<Vec3f> oa,ob;
                corridorOccupiedSearch(pos-Vec3f::Constant(1),pos+Vec3f::Constant(1),oa);
                other.corridorOccupiedSearch(pos-Vec3f::Constant(1),pos+Vec3f::Constant(1),ob);
                require(oa==ob,"BDM corridor mismatch");
            }
        }
    }
};
void checkRays() {
    std::mt19937 rng(8123);std::uniform_real_distribution<double>d(-20,20);
    for(double resolution:{.1,.15,.2}) for(int n=0;n<10000;++n) {
        Vec3f start(d(rng),d(rng),d(rng)),end(d(rng),d(rng),d(rng));
        if(n%3==0) start=(start/resolution).array().round().matrix()*resolution;
        if(n%5==0) end=(end/resolution).array().round().matrix()*resolution;
        if(n%7==0) end.x()=start.x();
        if(n%11==0) end=start;
        raycaster::RayCaster old(resolution),fast(resolution);
        require(old.setInput(start,end)==fast.setInput(start,end),"ray init mismatch");
        Vec3f p;Vec3i id,expected;
        for(;;) {
            bool a=old.step(p),b=fast.stepIndex(id);require(a==b,"ray length mismatch");
            if(!a)break;
            for(int k=0;k<3;++k) old.posToIndex(p[k],expected[k]);
            require(id==expected,"ray voxel mismatch");
        }
    }
}
template<class T> bool read(std::ifstream &f,T &x) { return bool(f.read(reinterpret_cast<char*>(&x),sizeof(x))); }
int main(int argc,char **argv) {
    try {
        if(argc!=2 && argc!=4)return 2;
        checkRays();
        std::mt19937 rng(42019),check_rng(51);
        if(argc==4) {
            BenchMap old(argv[1],false,false),fast(argv[1],true,false);
            std::ifstream in(argv[2],std::ios::binary);require(bool(in),"missing replay");
            std::ofstream csv(argv[3]);csv<<"frame,points,old_wall_ms,new_wall_ms,old_total_ms,new_total_ms,old_ray_ms,new_ray_ms,old_cache_ms,new_cache_ms\n";
            uint32_t count;size_t frame=0;
            while(read(in,count)) {
                double xyz[3];require(bool(in.read(reinterpret_cast<char*>(xyz),sizeof(xyz))),"truncated pose");
                Pose pose{Vec3f(xyz[0],xyz[1],xyz[2]),Eigen::Quaterniond::Identity()};
                PointCloud cloud;cloud.resize(count);
                for(auto &p:cloud) { float v[4];require(bool(in.read(reinterpret_cast<char*>(v),sizeof(v))),"truncated cloud");p.x=v[0];p.y=v[1];p.z=v[2];p.intensity=v[3]; }
                double a,b;
                auto run=[&](BenchMap &m,double &ms){auto t=std::chrono::steady_clock::now();m.updateProbMap(cloud,pose);ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t).count();};
                if(frame%2) {run(fast,b);run(old,a);}else{run(old,a);run(fast,b);}
                old.compare(fast,check_rng);
                auto &ta=old.timings();auto &tb=fast.timings();
                csv<<frame<<','<<count<<','<<a<<','<<b<<','<<ta[0]*1000<<','<<tb[0]*1000<<','<<ta[1]*1000<<','<<tb[1]*1000<<','<<ta[2]*1000<<','<<tb[2]*1000<<'\n';
                ++frame;
            }
            require(frame>0,"empty replay");
            std::cout<<"replay PASS frames="<<frame<<" stamp_bytes="<<fast.stampBytes()<<'\n';
        } else {
            size_t frames=0;
            for(int batch:{1,3})for(bool rays:{false,true}) {
                BenchMap old(argv[1],false,true,batch,rays),fast(argv[1],true,true,batch,rays);
                std::uniform_real_distribution<float> xy(-8,8),z(-2,6);
                for(int frame=0;frame<90;++frame) {
                    Vec3f origin(frame<30?0:(frame<60?-2:60),frame<60?0:-60,1.575);
                    origin.x()+=.15*(frame%7);
                    Pose pose{origin,Eigen::Quaterniond::Identity()};PointCloud cloud;
                    if(frame%13!=0)for(int n=0;n<1500;++n) {
                        PclPoint p;p.x=origin.x()+xy(rng);p.y=origin.y()+xy(rng);p.z=z(rng);p.intensity=1;
                        cloud.push_back(p);if(n%5==0)cloud.push_back(p);
                    }
                    if(frame==35)fast.rollover();
                    if(frame==40) {PclPoint p;p.x=std::numeric_limits<float>::quiet_NaN();p.y=0;p.z=1;p.intensity=1;cloud.push_back(p);}
                    old.updateProbMap(cloud,pose);fast.updateProbMap(cloud,pose);old.compare(fast,check_rng);++frames;
                }
            }
            std::cout<<"probmap_fusion_benchmark: PASS rays=30000 frames="<<frames<<" probability/evidence/delta mismatches=0\n";
        }
    } catch(const std::exception &e) {std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}
}
