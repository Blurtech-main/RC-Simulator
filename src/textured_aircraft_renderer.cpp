#include "textured_aircraft_renderer.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>

namespace {
struct P3{float x,y,z;};
P3 cross(P3 a,P3 b){return{a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
P3 rotate(float w,float x,float y,float z,P3 v){P3 q{x,y,z};P3 t=cross(q,v);t={2*t.x,2*t.y,2*t.z};P3 c=cross(q,t);return{v.x+w*t.x+c.x,v.y+w*t.y+c.y,v.z+w*t.z+c.z};}
float edge(float ax,float ay,float bx,float by,float px,float py){return(px-ax)*(by-ay)-(py-ay)*(bx-ax);}
template<class T>bool read(std::ifstream&f,T&v){return bool(f.read(reinterpret_cast<char*>(&v),sizeof(v)));}
}

TexturedAircraftRenderer& TexturedAircraftRenderer::instance(){static TexturedAircraftRenderer r;return r;}
TexturedAircraftRenderer::TexturedAircraftRenderer(){
  wchar_t exe[32768]{};GetModuleFileNameW(nullptr,exe,32768);auto path=std::filesystem::path(exe).parent_path()/L"assets"/L"aircraft"/L"cessna172"/L"cessna-172.rctmesh";
  std::ifstream f(path,std::ios::binary);if(!f){error_=L"missing assets/aircraft/cessna172/cessna-172.rctmesh";return;}
  char magic[8]{};f.read(magic,8);uint32_t version{},triangles{},materials{};float sourceScale{},span{},length{};
  if(std::string(magic,8)!="RCTMESH1"||!read(f,version)||!read(f,triangles)||!read(f,materials)||!read(f,sourceScale)||!read(f,span)||!read(f,length)||(version!=2&&version!=3)){error_=L"invalid Cessna runtime mesh";return;}
  if(triangles>1000000||materials>128){error_=L"unsafe Cessna runtime mesh counts";return;}
  textures_.resize(materials);
  for(auto&t:textures_){uint32_t bytes{};if(!read(f,t.flags)||!read(f,t.width)||!read(f,t.height)||!read(f,bytes)||!t.width||!t.height||bytes!=t.width*t.height*4u){error_=L"invalid Cessna texture";return;}t.rgba.resize(bytes);f.read(reinterpret_cast<char*>(t.rgba.data()),bytes);if(!f){error_=L"truncated Cessna texture";return;}}
  triangles_.resize(triangles);float propMinX=INFINITY,propMaxX=-INFINITY,propMinY=INFINITY,propMaxY=-INFINITY;for(auto&t:triangles_){for(auto&v:t.v){f.read(reinterpret_cast<char*>(&v),sizeof(v));}read(f,t.material);if(version>=3)read(f,t.part);if(!f||t.material>=materials||t.part>32){error_=L"truncated Cessna geometry";triangles_.clear();return;}if(t.part==1)for(const auto&v:t.v){propMinX=std::min(propMinX,v.x);propMaxX=std::max(propMaxX,v.x);propMinY=std::min(propMinY,v.y);propMaxY=std::max(propMaxY,v.y);}}
  if(std::isfinite(propMinX)&&std::isfinite(propMinY)){propellerHubX_=(propMinX+propMaxX)*.5f;propellerHubY_=(propMinY+propMaxY)*.5f;}
  pixels_.resize(480*340*4);depth_.resize(480*340);loaded_=true;
}

void TexturedAircraftRenderer::draw(HDC target,int cx,int cy,float ppm,float qw,float qx,float qy,float qz,float propellerAngle,float propellerRpm){
  if(!loaded_)return;constexpr int W=480,H=340;std::fill(pixels_.begin(),pixels_.end(),static_cast<unsigned char>(0));std::fill(depth_.begin(),depth_.end(),-1e30f);
  struct S{float x,y,z,nx,ny,nz,u,v;};
  for(int pass=0;pass<2;pass++)for(const auto&t:triangles_){const auto&tex=textures_[t.material];bool transparent=(tex.flags&1)!=0;if(transparent!=(pass==1))continue;S s[3]{};for(int i=0;i<3;i++){
    // The imported C172 mesh points down local -Z, while the flight model's
    // forward vector is local +Z.  Turn only the visual mesh around so a
    // fresh flight starts looking down the runway instead of back at camera.
    P3 local{t.v[i].x,t.v[i].y,t.v[i].z},localNormal{t.v[i].nx,t.v[i].ny,t.v[i].nz};
    if(t.part==1){float angle=propellerAngle;float ca=std::cos(angle),sa=std::sin(angle);float x=local.x-propellerHubX_,y=local.y-propellerHubY_;local={propellerHubX_+ca*x-sa*y,propellerHubY_+sa*x+ca*y,local.z};localNormal={ca*localNormal.x-sa*localNormal.y,sa*localNormal.x+ca*localNormal.y,localNormal.z};}
    auto p=rotate(qw,qx,qy,qz,{-local.x,local.y,-local.z});
    auto n=rotate(qw,qx,qy,qz,{-localNormal.x,localNormal.y,-localNormal.z});
    float k=1.f/(1.f+std::max(-.75f,p.z)*.12f);s[i]={W*.5f+p.x*ppm*k,H*.5f-(p.y-.16f*p.z)*ppm*k,p.z,n.x,n.y,n.z,t.v[i].u,t.v[i].v};
  }
  (void)propellerRpm; // Reserved for a calibrated shutter/motion-blur pass.
    float area=edge(s[0].x,s[0].y,s[1].x,s[1].y,s[2].x,s[2].y);if(std::abs(area)<.01f)continue;
    int x0=std::clamp(int(std::floor(std::min({s[0].x,s[1].x,s[2].x}))),0,W-1),x1=std::clamp(int(std::ceil(std::max({s[0].x,s[1].x,s[2].x}))),0,W-1);
    int y0=std::clamp(int(std::floor(std::min({s[0].y,s[1].y,s[2].y}))),0,H-1),y1=std::clamp(int(std::ceil(std::max({s[0].y,s[1].y,s[2].y}))),0,H-1);
    for(int y=y0;y<=y1;y++)for(int x=x0;x<=x1;x++){float w0=edge(s[1].x,s[1].y,s[2].x,s[2].y,x+.5f,y+.5f)/area,w1=edge(s[2].x,s[2].y,s[0].x,s[0].y,x+.5f,y+.5f)/area,w2=1-w0-w1;if(w0<0||w1<0||w2<0)continue;float z=w0*s[0].z+w1*s[1].z+w2*s[2].z;size_t pi=size_t(y)*W+x;if(z<=depth_[pi])continue;if(!transparent)depth_[pi]=z;
      float u=w0*s[0].u+w1*s[1].u+w2*s[2].u,v=w0*s[0].v+w1*s[1].v+w2*s[2].v;u-=std::floor(u);v-=std::floor(v);float fx=u*tex.width-.5f,fy=v*tex.height-.5f;int ix=int(std::floor(fx)),iy=int(std::floor(fy));float ax=fx-ix,ay=fy-iy;unsigned sample[4]{};for(int c=0;c<4;c++){auto at=[&](int px,int py){px=(px%int(tex.width)+int(tex.width))%int(tex.width);py=(py%int(tex.height)+int(tex.height))%int(tex.height);return tex.rgba[(size_t(py)*tex.width+px)*4+c];};float top=at(ix,iy)*(1-ax)+at(ix+1,iy)*ax,bottom=at(ix,iy+1)*(1-ax)+at(ix+1,iy+1)*ax;sample[c]=unsigned(top*(1-ay)+bottom*ay+.5f);}size_t di=pi*4;float ny=w0*s[0].ny+w1*s[1].ny+w2*s[2].ny;float nz=w0*s[0].nz+w1*s[1].nz+w2*s[2].nz;float light=.70f+.30f*std::abs(.82f*ny+.42f*nz);unsigned a=sample[3];unsigned srcB=unsigned(sample[2]*light),srcG=unsigned(sample[1]*light),srcR=unsigned(sample[0]*light);if(transparent){
        // The source glass is intentionally very transparent for a browser
        // environment. Against this simulator's blue sky it disappeared and
        // exposed the pale cabin interior, so retain its detail while giving
        // it physically legible blue-grey absorption and reflection.
        a=std::max(a,145u);srcR=(srcR*2+42)/3;srcG=(srcG*2+78)/3;srcB=(srcB*2+98)/3;
        unsigned inv=255-a;pixels_[di+0]=static_cast<unsigned char>((srcB*a+pixels_[di+0]*inv)/255);pixels_[di+1]=static_cast<unsigned char>((srcG*a+pixels_[di+1]*inv)/255);pixels_[di+2]=static_cast<unsigned char>((srcR*a+pixels_[di+2]*inv)/255);pixels_[di+3]=static_cast<unsigned char>(a+(pixels_[di+3]*inv)/255);
      }else{pixels_[di+0]=static_cast<unsigned char>(srcB*a/255);pixels_[di+1]=static_cast<unsigned char>(srcG*a/255);pixels_[di+2]=static_cast<unsigned char>(srcR*a/255);pixels_[di+3]=static_cast<unsigned char>(a);}
    }
  }
  BITMAPINFO bi{};bi.bmiHeader.biSize=sizeof(BITMAPINFOHEADER);bi.bmiHeader.biWidth=W;bi.bmiHeader.biHeight=-H;bi.bmiHeader.biPlanes=1;bi.bmiHeader.biBitCount=32;bi.bmiHeader.biCompression=BI_RGB;void*bits{};HBITMAP bm=CreateDIBSection(target,&bi,DIB_RGB_COLORS,&bits,nullptr,0);if(!bm)return;std::copy(pixels_.begin(),pixels_.end(),static_cast<unsigned char*>(bits));HDC src=CreateCompatibleDC(target);auto old=SelectObject(src,bm);BLENDFUNCTION blend{AC_SRC_OVER,0,255,AC_SRC_ALPHA};AlphaBlend(target,cx-W/2,cy-H/2,W,H,src,0,0,W,H,blend);SelectObject(src,old);DeleteDC(src);DeleteObject(bm);
}
