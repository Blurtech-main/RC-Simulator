#include <windows.h>
#include <dinput.h>
#include <mmsystem.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cwctype>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

constexpr float PI = 3.14159265358979323846f;
constexpr float DT = 1.0f / 120.0f;
constexpr int AXES = 8;
constexpr int CHANNELS = 4;
constexpr int ID_RECORD_FLIGHT = 1001;
constexpr int ID_STOP_FLIGHT = 1002;
constexpr int ID_CLEAR_FLIGHT = 1003;
constexpr int ID_MANUAL_OVERRIDE = 1004;

struct V3 { float x{}, y{}, z{}; };
static V3 operator+(V3 a,V3 b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
static V3 operator-(V3 a,V3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
static V3 operator*(V3 a,float s){return {a.x*s,a.y*s,a.z*s};}
static V3 operator/(V3 a,float s){return a*(1.0f/s);}
static V3& operator+=(V3& a,V3 b){a=a+b;return a;}
static float dot(V3 a,V3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
static float len(V3 a){return std::sqrt(dot(a,a));}
static V3 norm(V3 a){float l=len(a);return l>1e-5f?a/l:V3{};}
static float clampf(float v,float a,float b){return std::clamp(v,a,b);}

struct Quat { float w{1},x{},y{},z{}; };
static Quat qmul(Quat a,Quat b){return {a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w};}
static Quat qnorm(Quat q){float l=std::sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z);return {q.w/l,q.x/l,q.y/l,q.z/l};}
static V3 rotate(Quat q,V3 v){Quat r=qmul(qmul(q,{0,v.x,v.y,v.z}),{q.w,-q.x,-q.y,-q.z});return {r.x,r.y,r.z};}
static V3 invrot(Quat q,V3 v){return rotate({q.w,-q.x,-q.y,-q.z},v);}

struct AxisCal { long lo{-1000}, hi{1000}, center{}; float dead{0.035f}; };
struct Mapping { std::array<int,CHANNELS> axis{0,1,2,5}; std::array<bool,CHANNELS> reverse{false,true,false,false}; };
struct Controls { float ail{},ele{},thr{},rud{}; };

class Controller {
  IDirectInput8W* di{}; IDirectInputDevice8W* dev{}; HWND hwnd{}; unsigned retryTicks{};
  static BOOL CALLBACK enumCb(const DIDEVICEINSTANCEW* i,VOID* p){
    auto* self=static_cast<Controller*>(p); if(self->dev) return DIENUM_STOP;
    if(SUCCEEDED(self->di->CreateDevice(i->guidInstance,&self->dev,nullptr))) self->name=i->tszProductName;
    return self->dev?DIENUM_STOP:DIENUM_CONTINUE;
  }
public:
  std::wstring name=L"Keyboard fallback"; std::array<long,AXES> raw{}; std::array<AxisCal,AXES> cal{}; Mapping map; bool calibrating{};
  ~Controller(){if(dev){dev->Unacquire();dev->Release();}if(di)di->Release();}
  bool discover(){
    if(dev||!di)return dev!=nullptr; name=L"Keyboard fallback"; di->EnumDevices(DI8DEVCLASS_GAMECTRL,enumCb,this,DIEDFL_ATTACHEDONLY); if(!dev)return false;
    dev->SetDataFormat(&c_dfDIJoystick2); dev->SetCooperativeLevel(hwnd,DISCL_BACKGROUND|DISCL_NONEXCLUSIVE);
    DIPROPRANGE range{}; range.diph.dwSize=sizeof(range);range.diph.dwHeaderSize=sizeof(DIPROPHEADER);range.diph.dwHow=DIPH_DEVICE;range.lMin=-1000;range.lMax=1000;
    dev->SetProperty(DIPROP_RANGE,&range.diph); dev->Acquire(); return true;
  }
  bool init(HINSTANCE inst,HWND w){hwnd=w;if(FAILED(DirectInput8Create(inst,DIRECTINPUT_VERSION,IID_IDirectInput8W,(void**)&di,nullptr)))return false;return discover();}
  bool poll(){
    if(!dev){if(++retryTicks>=120){retryTicks=0;discover();}if(!dev)return false;} DIJOYSTATE2 s{}; HRESULT hr=dev->Poll(); if(FAILED(hr)){dev->Acquire();hr=dev->Poll();} if(FAILED(hr)||FAILED(dev->GetDeviceState(sizeof(s),&s)))return false;
    raw={s.lX,s.lY,s.lZ,s.lRx,s.lRy,s.lRz,s.rglSlider[0],s.rglSlider[1]};
    if(calibrating)for(int i=0;i<AXES;i++){cal[i].lo=std::min(cal[i].lo,raw[i]);cal[i].hi=std::max(cal[i].hi,raw[i]);}
    return true;
  }
  float value(int i) const {
    if(i<0||i>=AXES)return 0; const auto& c=cal[i]; float span=(raw[i]>=c.center)?float(std::max(1L,c.hi-c.center)):float(std::max(1L,c.center-c.lo));
    float v=clampf(float(raw[i]-c.center)/span,-1,1); if(std::abs(v)<c.dead)return 0; return std::copysign((std::abs(v)-c.dead)/(1-c.dead),v);
  }
  Controls controls() const {float v[4]{};for(int i=0;i<4;i++){v[i]=value(map.axis[i])*(map.reverse[i]?-1.f:1.f);}return {v[0],v[1],clampf((v[2]+1)*.5f,0,1),v[3]};}
  void beginCal(){calibrating=true;for(auto& c:cal){c.lo=100000;c.hi=-100000;c.center=0;}}
  void endCal(){for(int i=0;i<AXES;i++){if(cal[i].hi<=cal[i].lo){cal[i].lo=-1000;cal[i].hi=1000;}cal[i].center=raw[i];}calibrating=false;save();}
  void load(){std::ifstream f("profiles/default.ini");if(!f)return;for(int i=0;i<AXES;i++)f>>cal[i].lo>>cal[i].hi>>cal[i].center>>cal[i].dead;for(int i=0;i<4;i++){int r;f>>map.axis[i]>>r;map.reverse[i]=r!=0;}}
  void save(){CreateDirectoryW(L"profiles",nullptr);std::ofstream f("profiles/default.ini");for(auto& c:cal)f<<c.lo<<' '<<c.hi<<' '<<c.center<<' '<<c.dead<<'\n';for(int i=0;i<4;i++)f<<map.axis[i]<<' '<<map.reverse[i]<<'\n';}
};

struct Plane {
  V3 p{0,.45f,0},v{},omega{}; Quat q{}; bool crashed{}; float aoa{},airspeed{},lift{},drag{},thrust{};
  void reset(){*this=Plane{};}
  void step(Controls c,float dt){
    if(crashed)return;
    V3 forward=rotate(q,{0,0,1}),up=rotate(q,{0,1,0}); V3 airBody=invrot(q,v); airspeed=len(v);
    aoa=std::atan2(-airBody.y,std::max(0.2f,airBody.z));
    constexpr float mass=2.2f,rho=1.225f,area=.62f;
    float cl=clampf(4.4f*aoa,-1.1f,1.25f); if(std::abs(aoa)>.30f)cl*=clampf(.55f-(std::abs(aoa)-.30f),.12f,.55f);
    lift=.5f*rho*airspeed*airspeed*area*cl; float cd=.045f+.07f*cl*cl; drag=.5f*rho*airspeed*airspeed*area*cd; thrust=18.f*c.thr;
    V3 force={0,-9.81f*mass,0}; force+=up*lift; if(airspeed>.1f)force+=norm(v)*-drag; force+=forward*thrust;
    v+=force/mass*dt; p+=v*dt;
    float authority=clampf(airspeed/12.f,.15f,1.4f); V3 torque={-c.ele*2.1f*authority,c.rud*.9f*authority,-c.ail*2.5f*authority};
    omega+=(torque-omega*1.5f)*dt; Quat dq={0,omega.x,omega.y,omega.z}; q=qnorm({q.w+.5f*qmul(q,dq).w*dt,q.x+.5f*qmul(q,dq).x*dt,q.y+.5f*qmul(q,dq).y*dt,q.z+.5f*qmul(q,dq).z*dt});
    if(p.y<.45f){if(std::abs(v.y)>4.5f||std::abs(rotate(q,{0,1,0}).y)<.55f)crashed=true;p.y=.45f;v.y=std::max(0.f,-v.y*.18f);v.x*=.9985f;v.z*=.9985f;}
  }
};

struct Lesson {
  enum Stage { TAKEOFF_ROLL, CLIMB, CRUISE, POWER_OFF_GLIDE, LAND, COMPLETE } stage{TAKEOFF_ROLL};
  float elapsed{}, maxAltitude{}, maxDistance{}; int score{1000}; bool failed{};
  void reset(){*this=Lesson{};}
  const wchar_t* objective() const {
    switch(stage){
      case TAKEOFF_ROLL:return L"Take off: build speed and leave the runway";
      case CLIMB:return L"Climb: reach 12 m altitude";
      case CRUISE:return L"Cruise: fly 120 m down the field";
      case POWER_OFF_GLIDE:return L"Glide: close throttle and descend below 6 m";
      case LAND:return L"Land: touch down upright below 10 m/s";
      default:return failed?L"Lesson failed - F3 to retry":L"Lesson complete - F3 to fly again";
    }
  }
  void update(const Plane& p,Controls c,float dt){
    if(stage==COMPLETE)return; elapsed+=dt;maxAltitude=std::max(maxAltitude,p.p.y);maxDistance=std::max(maxDistance,p.p.z);
    if(p.crashed){score=std::max(0,score-350);failed=true;stage=COMPLETE;return;}
    switch(stage){
      case TAKEOFF_ROLL:if(p.p.y>2.5f&&p.airspeed>7.f)stage=CLIMB;break;
      case CLIMB:if(p.p.y>=12.f)stage=CRUISE;break;
      case CRUISE:if(p.p.z>=120.f)stage=POWER_OFF_GLIDE;break;
      case POWER_OFF_GLIDE:if(c.thr<.15f&&p.v.y<-.2f&&p.p.y<6.f)stage=LAND;break;
      case LAND:if(p.p.y<=.46f&&p.airspeed<10.f){score=std::max(0,score-int(elapsed*2));stage=COMPLETE;}break;
      default:break;
    }
  }
};

struct Frame {uint64_t n{};Controls c;Plane p;};
class Replay {
public:
  bool recording{},playing{};std::ofstream out;std::vector<Frame> frames;size_t at{};
  bool startRecord(){stop();frames.clear();CreateDirectoryW(L"replays",nullptr);out.clear();out.open("replays/latest.csv",std::ios::trunc);out<<"frame,ail,ele,thr,rud,px,py,pz,vx,vy,vz,qw,qx,qy,qz,omega_x,omega_y,omega_z,crashed,aoa,airspeed,lift,drag,thrust\n";recording=bool(out);return recording;}
  void stopRecord(){if(out.is_open())out.close();recording=false;}
  void stop(){stopRecord();playing=false;at=0;}
  void clear(){stop();frames.clear();DeleteFileW(L"replays/latest.csv");}
  void write(uint64_t n,Controls c,const Plane&p){if(recording)out<<n<<','<<std::setprecision(9)<<c.ail<<','<<c.ele<<','<<c.thr<<','<<c.rud<<','<<p.p.x<<','<<p.p.y<<','<<p.p.z<<','<<p.v.x<<','<<p.v.y<<','<<p.v.z<<','<<p.q.w<<','<<p.q.x<<','<<p.q.y<<','<<p.q.z<<','<<p.omega.x<<','<<p.omega.y<<','<<p.omega.z<<','<<p.crashed<<','<<p.aoa<<','<<p.airspeed<<','<<p.lift<<','<<p.drag<<','<<p.thrust<<'\n';}
  bool load(){stop();std::ifstream f("replays/latest.csv");if(!f)return false;frames.clear();std::string line;std::getline(f,line);while(std::getline(f,line)){std::replace(line.begin(),line.end(),',',' ');std::istringstream s(line);Frame x;s>>x.n>>x.c.ail>>x.c.ele>>x.c.thr>>x.c.rud>>x.p.p.x>>x.p.p.y>>x.p.p.z>>x.p.v.x>>x.p.v.y>>x.p.v.z>>x.p.q.w>>x.p.q.x>>x.p.q.y>>x.p.q.z;if(!s)continue;s>>x.p.omega.x>>x.p.omega.y>>x.p.omega.z>>x.p.crashed>>x.p.aoa>>x.p.airspeed>>x.p.lift>>x.p.drag>>x.p.thrust;frames.push_back(x);}at=0;playing=!frames.empty();return playing;}
  const Frame* next(){if(!playing||frames.empty())return nullptr;if(at>=frames.size())at=0;return &frames[at++];}
};

class AudioPpm {
  static constexpr int BUFFERS=4,SAMPLES=2048;
  struct Candidate {int threshold{},polarity{1},channel{},lastCount{},good{};bool high{};uint64_t lastEdge{},lastSync{};std::array<float,8> building{};};
  HWAVEIN in{}; std::array<WAVEHDR,BUFFERS> hdr{}; std::array<std::array<int16_t,SAMPLES>,BUFFERS> data{};
  std::mutex lock; std::array<float,8> live{};std::array<Candidate,14> candidates{};uint64_t sampleCounter{},lastFrame{},lastHit{},lastLoggedHit{};unsigned frames{};float meter{};std::atomic_bool running{};
  UINT deviceIndex{};std::wstring source=L"No audio input";std::array<float,4> filtered{};
  void logHit(const Candidate& c,float frameMs){SYSTEMTIME t{};GetLocalTime(&t);std::ofstream f("ppm-hits.log",std::ios::app);f<<std::setfill('0')<<std::setw(4)<<t.wYear<<'-'<<std::setw(2)<<t.wMonth<<'-'<<std::setw(2)<<t.wDay<<' '<<std::setw(2)<<t.wHour<<':'<<std::setw(2)<<t.wMinute<<':'<<std::setw(2)<<t.wSecond<<'.'<<std::setw(3)<<t.wMilliseconds<<" PPM_HIT threshold="<<c.threshold<<" polarity="<<(c.polarity>0?"positive":"negative")<<" channels="<<c.channel<<" frame_ms="<<std::setprecision(4)<<frameMs<<'\n';}
  static void CALLBACK callback(HWAVEIN,UINT msg,DWORD_PTR instance,DWORD_PTR p1,DWORD_PTR){
    if(msg!=WIM_DATA)return;auto* self=reinterpret_cast<AudioPpm*>(instance);auto* h=reinterpret_cast<WAVEHDR*>(p1);self->process(reinterpret_cast<int16_t*>(h->lpData),h->dwBytesRecorded/2);if(self->running){h->dwBytesRecorded=0;waveInAddBuffer(self->in,h,sizeof(*h));}
  }
  void process(const int16_t* s,size_t count){
    int peak=0;for(size_t i=0;i<count;i++)peak=std::max(peak,std::abs(int(s[i])));{std::scoped_lock g(lock);meter=std::max(float(peak)/32768.f,meter*.82f);}
    for(size_t i=0;i<count;i++){
      ++sampleCounter;for(auto& c:candidates){int v=int(s[i])*c.polarity;bool now=v>c.threshold;if(!now&&v<c.threshold/3)c.high=false;if(now&&!c.high){c.high=true;if(c.lastEdge){float us=float(sampleCounter-c.lastEdge)*1000000.f/48000.f;if(us>=3000&&us<=15000){float frameMs=c.lastSync?float(sampleCounter-c.lastSync)*1000.f/48000.f:0;bool plausible=c.channel>=4&&c.channel<=8&&frameMs>=12&&frameMs<=35;bool valid=plausible&&c.channel==c.lastCount&&frameMs>=15&&frameMs<=30;if(plausible){uint64_t nowTick=GetTickCount64();{std::scoped_lock g(lock);lastHit=nowTick;}if(nowTick-lastLoggedHit>500){logHit(c,frameMs);lastLoggedHit=nowTick;}}c.good=valid?std::min(c.good+1,20):0;if(c.good>=5){std::scoped_lock g(lock);live=c.building;lastFrame=GetTickCount64();++frames;}c.lastCount=c.channel;c.lastSync=sampleCounter;c.channel=0;}else if(us>=700&&us<=2300&&c.channel<8){c.building[c.channel++]=clampf((us-1500.f)/500.f,-1,1);}else if(us>2300){c.channel=0;c.good=0;}}c.lastEdge=sampleCounter;}}
      }
    }
public:
  std::array<float,8> channels() {std::scoped_lock g(lock);return live;}
  float audioLevel(){std::scoped_lock g(lock);float v=meter;meter*=.92f;return v;}
  bool active() {std::scoped_lock g(lock);return frames>=3&&GetTickCount64()-lastFrame<750;}
  bool hit(){std::scoped_lock g(lock);return lastHit&&GetTickCount64()-lastHit<900;}
  Controls controls(){
    auto c=channels();float raw[4]={c[1],-c[2],c[0],c[3]};
    auto stable=[](float v){constexpr float dead=.065f;float a=std::abs(v);if(a<=dead)return 0.f;return std::copysign((a-dead)/(1.f-dead),v);};
    for(int i=0;i<4;i++){float target=stable(raw[i]);filtered[i]+=(target-filtered[i])*.18f;}
    return {filtered[0],filtered[1],clampf((filtered[2]+1)*.5f,0,1),filtered[3]};
  }
  void resetDecoder(){
    std::scoped_lock g(lock);live={};filtered={};candidates={};sampleCounter=0;lastFrame=0;lastHit=0;lastLoggedHit=0;frames=0;meter=0;
    const int levels[7]={400,700,1100,1700,2600,4000,6500};for(int i=0;i<14;i++){candidates[i].threshold=levels[i%7];candidates[i].polarity=i<7?1:-1;}
  }
  void stop(){
    if(!in)return;running=false;waveInStop(in);waveInReset(in);for(auto& h:hdr)waveInUnprepareHeader(in,&h,sizeof(h));waveInClose(in);in=nullptr;hdr={};
  }
  bool open(UINT device){
    stop();resetDecoder();UINT count=waveInGetNumDevs();if(!count){source=L"No audio input";return false;}deviceIndex=device%count;WAVEINCAPSW caps{};if(waveInGetDevCapsW(deviceIndex,&caps,sizeof(caps))==MMSYSERR_NOERROR)source=caps.szPname;else source=L"Audio input "+std::to_wstring(deviceIndex+1);
    WAVEFORMATEX f{};f.wFormatTag=WAVE_FORMAT_PCM;f.nChannels=1;f.nSamplesPerSec=48000;f.wBitsPerSample=16;f.nBlockAlign=2;f.nAvgBytesPerSec=96000;
    if(waveInOpen(&in,deviceIndex,&f,reinterpret_cast<DWORD_PTR>(callback),reinterpret_cast<DWORD_PTR>(this),CALLBACK_FUNCTION)!=MMSYSERR_NOERROR){source+=L" (open failed)";in=nullptr;return false;}
    running=true;for(int i=0;i<BUFFERS;i++){hdr[i].lpData=reinterpret_cast<LPSTR>(data[i].data());hdr[i].dwBufferLength=SAMPLES*2;waveInPrepareHeader(in,&hdr[i],sizeof(WAVEHDR));waveInAddBuffer(in,&hdr[i],sizeof(WAVEHDR));}waveInStart(in);return true;
  }
  bool init(){
    {std::ofstream f("ppm-hits.log",std::ios::app);f<<"--- RC Trainer PPM sniffer started ---\n";}
    UINT count=waveInGetNumDevs();
    if(!count)return false;
    // Prefer a Native Instruments Komplete Audio interface when it is available.
    // Otherwise retain the normal default choice: the last Windows capture device.
    for(UINT i=0;i<count;i++){
      WAVEINCAPSW caps{};
      if(waveInGetDevCapsW(i,&caps,sizeof(caps))!=MMSYSERR_NOERROR)continue;
      std::wstring name=caps.szPname;
      std::transform(name.begin(),name.end(),name.begin(),[](wchar_t ch){return static_cast<wchar_t>(std::towlower(ch));});
      if(name.find(L"komplete audio")!=std::wstring::npos)return open(i);
    }
    return open(count-1);
  }
  bool nextSource(){UINT count=waveInGetNumDevs();return count&&open((deviceIndex+1)%count);}
  std::wstring sourceName(){return source;}
  ~AudioPpm(){stop();}
};

struct App {
  HWND wnd{},recordButton{},stopButton{},clearButton{},overrideCheck{};HINSTANCE inst{};Controller ctrl;AudioPpm ppm;Plane plane;Replay replay;Lesson lesson;bool help{true};int selected{};uint64_t frame{};double accum{},paintAccum{},crashResetElapsed{};LARGE_INTEGER last{},freq{};
  static App* self(HWND w){return reinterpret_cast<App*>(GetWindowLongPtrW(w,GWLP_USERDATA));}
  static LRESULT CALLBACK proc(HWND w,UINT m,WPARAM wp,LPARAM lp){
    if(m==WM_NCCREATE)SetWindowLongPtrW(w,GWLP_USERDATA,(LONG_PTR)((CREATESTRUCT*)lp)->lpCreateParams);App* a=self(w);if(!a)return DefWindowProcW(w,m,wp,lp);
    if(m==WM_DESTROY){PostQuitMessage(0);return 0;}if(m==WM_PAINT){a->paint();return 0;}if(m==WM_KEYDOWN){a->key((int)wp);return 0;}if(m==WM_MOUSEWHEEL){a->scrollRightStick(GET_WHEEL_DELTA_WPARAM(wp));return 0;}if(m==WM_COMMAND){a->command(LOWORD(wp),HIWORD(wp));return 0;}return DefWindowProcW(w,m,wp,lp);
  }
  bool init(HINSTANCE h){inst=h;QueryPerformanceFrequency(&freq);QueryPerformanceCounter(&last);WNDCLASSW c{};c.lpfnWndProc=proc;c.hInstance=h;c.lpszClassName=L"RCTrainerSlice";c.hCursor=LoadCursor(nullptr,IDC_ARROW);c.hbrBackground=(HBRUSH)GetStockObject(BLACK_BRUSH);RegisterClassW(&c);wnd=CreateWindowW(c.lpszClassName,L"RC Trainer - DX6 to flying trainer",WS_OVERLAPPEDWINDOW|WS_VISIBLE,CW_USEDEFAULT,CW_USEDEFAULT,1280,800,nullptr,nullptr,h,this);if(!wnd)return false;
    recordButton=CreateWindowW(L"BUTTON",L"RECORD FLIGHT",WS_CHILD|WS_VISIBLE|BS_PUSHBUTTON,20,260,135,28,wnd,(HMENU)(INT_PTR)ID_RECORD_FLIGHT,h,nullptr);
    stopButton=CreateWindowW(L"BUTTON",L"STOP",WS_CHILD|WS_VISIBLE|BS_PUSHBUTTON,165,260,80,28,wnd,(HMENU)(INT_PTR)ID_STOP_FLIGHT,h,nullptr);
    clearButton=CreateWindowW(L"BUTTON",L"CLEAR",WS_CHILD|WS_VISIBLE|BS_PUSHBUTTON,255,260,80,28,wnd,(HMENU)(INT_PTR)ID_CLEAR_FLIGHT,h,nullptr);
    overrideCheck=CreateWindowW(L"BUTTON",L"MANUAL OVERRIDE",WS_CHILD|WS_VISIBLE|BS_AUTOCHECKBOX,345,260,170,28,wnd,(HMENU)(INT_PTR)ID_MANUAL_OVERRIDE,h,nullptr);
    ctrl.init(h,wnd);ctrl.load();bool audio=ppm.init();replay.load();std::ofstream diagnostic("controller-diagnostic.txt");diagnostic<<"AUDIO_PPM_CAPTURE="<<(audio?"OPEN":"FAILED")<<"\nDIRECTINPUT="<<(ctrl.name==L"Keyboard fallback"?"NONE":"FOUND")<<"\nGROUND_RENDERER=PROCEDURAL\n";return true;}
  float keyboardThrottle{},keyboardElevator{};
  Controls keyboard(){Controls c{};c.ail=float((GetAsyncKeyState(VK_RIGHT)<0)-(GetAsyncKeyState(VK_LEFT)<0));c.ele=keyboardElevator;c.thr=keyboardThrottle;c.rud=float((GetAsyncKeyState('D')<0)-(GetAsyncKeyState('A')<0));keyboardThrottle=clampf(keyboardThrottle+float((GetAsyncKeyState('W')<0)-(GetAsyncKeyState('S')<0))*DT*.45f,0,1);return c;}
  void scrollRightStick(short delta){keyboardElevator=clampf(keyboardElevator+float(delta)/WHEEL_DELTA*.1f,-1.f,1.f);}
  void startRecording(){replay.startRecord();frame=0;SendMessageW(overrideCheck,BM_SETCHECK,BST_UNCHECKED,0);}
  void stopFlight(){replay.stop();}
  void command(int id,int event){if(event!=BN_CLICKED)return;if(id==ID_RECORD_FLIGHT)startRecording();else if(id==ID_STOP_FLIGHT)stopFlight();else if(id==ID_CLEAR_FLIGHT)replay.clear();else if(id==ID_MANUAL_OVERRIDE&&SendMessageW(overrideCheck,BM_GETCHECK,0,0)==BST_CHECKED)replay.playing=false;}
  void key(int k){
    if(k==VK_ESCAPE)DestroyWindow(wnd);else if(k==VK_F1)help=!help;else if(k==VK_F2){ppm.nextSource();}else if(k=='C'){if(ctrl.calibrating)ctrl.endCal();else ctrl.beginCal();}
    else if(k==VK_RETURN&&ctrl.calibrating)ctrl.endCal();else if(k>='1'&&k<='4')selected=k-'1';
    else if(k==VK_OEM_4){ctrl.map.axis[selected]=(ctrl.map.axis[selected]+AXES-1)%AXES;ctrl.save();}
    else if(k==VK_OEM_6){ctrl.map.axis[selected]=(ctrl.map.axis[selected]+1)%AXES;ctrl.save();}
    else if(k=='R'){ctrl.map.reverse[selected]=!ctrl.map.reverse[selected];ctrl.save();}
    else if(k==VK_OEM_MINUS||k==VK_OEM_PLUS){auto& d=ctrl.cal[ctrl.map.axis[selected]].dead;d=clampf(d+(k==VK_OEM_PLUS?.005f:-.005f),0,.25f);ctrl.save();}
    else if(k==VK_F3){plane.reset();lesson.reset();frame=0;}else if(k==VK_F5){if(replay.recording)replay.stopRecord();else startRecording();}else if(k==VK_F6)replay.load();else if(k==VK_F7)stopFlight();else if(k==VK_SPACE){plane.reset();lesson.reset();frame=0;}
  }
  void tick(){
    LARGE_INTEGER now;QueryPerformanceCounter(&now);double elapsed=double(now.QuadPart-last.QuadPart)/freq.QuadPart;last=now;elapsed=std::min(elapsed,.1);accum+=elapsed;paintAccum+=elapsed;ctrl.poll();
    while(accum>=DT){Controls c{};if(replay.playing){if(const Frame* saved=replay.next()){c=saved->c;plane=saved->p;}}else{c=ppm.active()?ppm.controls():(ctrl.name==L"Keyboard fallback"?keyboard():ctrl.controls());plane.step(c,DT);lesson.update(plane,c,DT);if(plane.crashed){crashResetElapsed+=DT;if(crashResetElapsed>=2.0){plane.reset();lesson.reset();frame=0;crashResetElapsed=0;}}else crashResetElapsed=0;replay.write(frame,c,plane);}frame++;accum-=DT;}if(paintAccum>=1.0/60.0){paintAccum=std::fmod(paintAccum,1.0/60.0);InvalidateRect(wnd,nullptr,FALSE);}
  }
  static void txt(HDC dc,int x,int y,const std::wstring&s,COLORREF col=RGB(235,240,240)){SetTextColor(dc,col);SetBkMode(dc,TRANSPARENT);TextOutW(dc,x,y,s.c_str(),(int)s.size());}
  static std::wstring f1(float v){wchar_t b[32];swprintf_s(b,L"%.1f",v);return b;}
  void paint(){
    PAINTSTRUCT ps;HDC dc=BeginPaint(wnd,&ps);RECT rc;GetClientRect(wnd,&rc);HDC mem=CreateCompatibleDC(dc);HBITMAP bm=CreateCompatibleBitmap(dc,rc.right,rc.bottom);auto old=SelectObject(mem,bm);
    HBRUSH sky=CreateSolidBrush(RGB(94,157,207));FillRect(mem,&rc,sky);DeleteObject(sky);
    // A quaternion's individual components may change sign without changing its
    // orientation.  Use the aircraft's actual forward direction for a stable,
    // camera-like horizon, and use altitude in every ground-plane projection.
    V3 cameraForward=rotate(plane.q,{0,0,1});float cameraHeight=std::max(.45f,plane.p.y);
    // Keep the aircraft readable against the field: pitch is a small camera
    // cue, while altitude is what makes the ground recede.  A large pitch
    // camera shift can push the field off-screen and make ground contact look
    // like an unexplained mid-air crash.
    int horizon=int(rc.bottom*.48f+clampf(cameraForward.y,-.7f,.7f)*rc.bottom*.10f);
    POINT ground[4]={{0,horizon},{rc.right,horizon},{rc.right,rc.bottom},{0,rc.bottom}};HBRUSH grassFill=CreateSolidBrush(RGB(65,111,57));auto ob=SelectObject(mem,grassFill);Polygon(mem,ground,4);SelectObject(mem,ob);DeleteObject(grassFill);
    int vanishingX=rc.right/2;float zPhase=std::fmod(std::max(0.f,plane.p.z),5.f);
    HPEN grid=CreatePen(PS_SOLID,1,RGB(103,145,73));auto op=SelectObject(mem,grid);
    // Keep the field legible well beyond the immediate circuit: the previous
    // 120 m grid ended abruptly and the grass faded before the far field.
    for(int i=1;i<=96;i++){float d=i*5.f-zPhase;if(d<1.f)continue;int y=horizon+int(1050.f*cameraHeight/d);if(y>horizon&&y<rc.bottom){MoveToEx(mem,0,y,nullptr);LineTo(mem,rc.right,y);}}
    for(int i=-30;i<=30;i++){MoveToEx(mem,vanishingX,horizon,nullptr);LineTo(mem,vanishingX+i*115,rc.bottom);}SelectObject(mem,op);DeleteObject(grid);
    HPEN tufts=CreatePen(PS_SOLID,3,RGB(20,72,26));op=SelectObject(mem,tufts);
    auto grassHash=[](int x,int z){uint32_t h=uint32_t(x)*0x8da6b343u^uint32_t(z)*0xd8163841u;h^=h>>13;h*=0x85ebca6bu;return h^(h>>16);};
    float worldZ=std::max(0.f,plane.p.z),xPhase=std::fmod(plane.p.x,3.f),zPhaseGrass=std::fmod(worldZ,2.f);int baseX=int(std::floor(plane.p.x/3.f)),baseZ=int(std::floor(worldZ/2.f));
    for(int zi=1;zi<=480;zi++){float rowD=zi*2.f-zPhaseGrass;if(rowD<2.f)continue;float remaining=clampf(1.f-rowD/1050.f,.04f,1.f);float density=remaining*remaining*remaining/3.f;for(int xi=-96;xi<=96;xi++){uint32_t h=grassHash(baseX+xi,baseZ+zi);if(float(h&0xffffu)/65535.f>density)continue;float jitterX=(float((h>>16)&255u)/255.f-.5f)*2.5f,jitterD=(float((h>>24)&255u)/255.f-.5f)*1.4f,d=rowD+jitterD;if(d<1.f)continue;int y=horizon+int(920.f*cameraHeight/d);if(y<=horizon||y>=rc.bottom)continue;float wx=xi*3.f-xPhase+jitterX;int x=vanishingX+int(wx*150.f/d);if(x<0||x>=rc.right)continue;int oldSize=std::clamp(int(32.f*cameraHeight/d),2,10),bladeHeight=oldSize*8,halfWidth=std::max(1,oldSize/2);MoveToEx(mem,x-halfWidth,y,nullptr);LineTo(mem,x,y-bladeHeight);LineTo(mem,x+halfWidth,y);}}
    SelectObject(mem,op);DeleteObject(tufts);
    int cx=rc.right/2;float agl=std::max(0.f,plane.p.y-.45f);int cy=horizon+10-int(std::min(105.f,agl*5.25f));float bank=2*std::atan2(plane.q.z,plane.q.w);float cs=std::cos(bank),sn=std::sin(bank);auto pt=[&](float x,float y){return POINT{cx+int(x*cs-y*sn),cy+int(x*sn+y*cs)};};
    // The old screen-space silhouette only used bank, so elevator could move
    // the simulated aircraft without ever moving its visible nose.  Pitch is
    // the vertical component of its real forward vector: make that explicit
    // in the fuselage and tail geometry, rather than using an Euler component
    // (which would be unstable once the trainer has banked).
    float visualPitch=std::asin(clampf(cameraForward.y,-1.f,1.f));
    float pitchCue=clampf(visualPitch,-.72f,.72f)*78.f;
    float wingSpan=115.f*(1.f-.16f*std::abs(visualPitch));
    POINT wing[5]={pt(-wingSpan,0),pt(-12,-9),pt(0,-18),pt(12,-9),pt(wingSpan,0)};HPEN pen=CreatePen(PS_SOLID,8,RGB(250,225,45));op=SelectObject(mem,pen);Polyline(mem,wing,5);
    POINT fus[2]={pt(0,-28-pitchCue),pt(0,42+pitchCue*.38f)};Polyline(mem,fus,2);
    POINT tail[2]={pt(-35,32+pitchCue*.38f),pt(35,32+pitchCue*.38f)};Polyline(mem,tail,2);SelectObject(mem,op);DeleteObject(pen);
    HFONT font=CreateFontW(18,0,0,0,FW_NORMAL,FALSE,FALSE,FALSE,DEFAULT_CHARSET,OUT_DEFAULT_PRECIS,CLIP_DEFAULT_PRECIS,CLEARTYPE_QUALITY,FIXED_PITCH,L"Consolas");auto of=SelectObject(mem,font);
    bool ppmLive=ppm.active(),ppmHit=ppm.hit();auto ppmCh=ppm.channels();float mic=ppm.audioLevel();std::wstring input=ppmLive?L"DX6i 3.5mm PPM":ctrl.name;std::wstring mode=replay.playing?L"PLAYBACK":replay.recording?L"RECORDING":L"LIVE";txt(mem,20,18,L"RC TRAINER  |  "+mode,RGB(255,225,80));txt(mem,20,44,L"Control: "+input);txt(mem,20,70,L"Audio source [F2]: "+ppm.sourceName(),RGB(255,225,80));txt(mem,20,96,L"Speed "+f1(plane.airspeed)+L" m/s   Alt AGL "+f1(std::max(0.f,plane.p.y-.45f))+L" m   Pitch "+f1(visualPitch*180/PI)+L" deg   AoA "+f1(plane.aoa*180/PI)+L" deg");txt(mem,20,122,L"Lift "+f1(plane.lift)+L" N   Drag "+f1(plane.drag)+L" N   Thrust "+f1(plane.thrust)+L" N");txt(mem,20,148,L"AUDIO SIGNAL",RGB(255,225,80));Rectangle(mem,155,149,375,165);COLORREF meterCol=ppmLive?RGB(70,230,100):(ppmHit?RGB(255,225,50):RGB(255,160,45));HBRUSH mb=CreateSolidBrush(meterCol);RECT mr{157,151,157+int(clampf(mic,0,1)*216),163};FillRect(mem,&mr,mb);DeleteObject(mb);txt(mem,385,147,ppmLive?L"PPM LOCKED":(ppmHit?L"PPM HIT":L"move sticks"),meterCol);
    txt(mem,20,180,L"FLIGHT SCHOOL  "+std::to_wstring(std::min(int(lesson.stage)+1,5))+L"/5",RGB(255,225,80));txt(mem,20,206,lesson.objective(),lesson.failed?RGB(255,90,70):RGB(245,245,245));txt(mem,20,232,L"Score "+std::to_wstring(lesson.score)+L"   Time "+f1(lesson.elapsed)+L" s   F3 restart");
    if(plane.crashed)txt(mem,cx-135,130,L"CRASH — RESTARTING IN "+f1(std::max(0.f,2.f-float(crashResetElapsed)))+L" s",RGB(255,90,70));
    const wchar_t* names[4]={L"AIL",L"ELE",L"THR",L"RUD"};for(int i=0;i<4;i++){int y=rc.bottom-130+i*25;std::wstring s=(i==selected?L"> ":L"  ")+std::wstring(names[i])+L" axis "+std::to_wstring(ctrl.map.axis[i]+1)+(ctrl.map.reverse[i]?L" REV":L"");txt(mem,20,y,s,i==selected?RGB(255,225,80):RGB(235,240,240));}
    for(int i=0;i<AXES;i++){int x=rc.right-300,y=20+i*24;Rectangle(mem,x,y,x+220,y+15);float av=ppmLive?ppmCh[i]:ctrl.value(i);int v=int((av+1)*.5f*216);HBRUSH bar=CreateSolidBrush(RGB(255,195,45));RECT br{x+2,y+2,x+2+v,y+13};FillRect(mem,&br,bar);DeleteObject(bar);txt(mem,x-72,y-2,(ppmLive?L"PPM":L"AX")+std::to_wstring(i+1)+L" "+f1(av));}
    if(ctrl.calibrating)txt(mem,cx-265,210,L"CALIBRATING: move all sticks fully, centre them, press ENTER",RGB(255,225,80));
    if(help){RECT box{cx-330,300,cx+330,600};HBRUSH bb=CreateSolidBrush(RGB(20,30,35));FillRect(mem,&box,bb);DeleteObject(bb);std::array<std::wstring,10> l={L"SECOND SLICE CONTROLS",L"Record/Stop saves the exact flight; Clear removes it",L"Saved flight loops automatically on the next launch",L"Manual Override stops playback and restores live control",L"F2: choose audio-interface input",L"F3: restart flight-school lesson",L"C: calibrate   ENTER: finish at stick centres",L"SPACE: runway reset   ESC: exit",L"Keyboard: W/S left stick vertical; A/D left stick horizontal",L"Mouse wheel: right stick vertical; arrows: right stick horizontal"};for(size_t i=0;i<l.size();i++)txt(mem,cx-300,320+(int)i*27,l[i],i==0?RGB(255,225,80):RGB(235,240,240));}
    SelectObject(mem,of);DeleteObject(font);BitBlt(dc,0,0,rc.right,rc.bottom,mem,0,0,SRCCOPY);SelectObject(mem,old);DeleteObject(bm);DeleteDC(mem);EndPaint(wnd,&ps);
  }
};

static int selfTest(){
  Plane a,b;uint64_t hashA=1469598103934665603ull,hashB=hashA;auto run=[](Plane&p,uint64_t& h){for(int i=0;i<2400;i++){Controls c{.18f*std::sin(i*.013f),.08f*std::sin(i*.021f),i<300?i/300.f:.72f,.04f};p.step(c,DT);const unsigned char* q=(const unsigned char*)&p;for(size_t k=0;k<sizeof(p);k++){h^=q[k];h*=1099511628211ull;}}};run(a,hashA);run(b,hashB);bool finite=std::isfinite(a.p.x)&&std::isfinite(a.p.y)&&std::isfinite(a.p.z);bool deterministic=hashA==hashB;
  Plane takeoff;float peak=takeoff.p.y;for(int i=0;i<1200&&!takeoff.crashed;i++){Controls c{};c.thr=1.f;c.ele=i>240?.12f:0.f;takeoff.step(c,DT);peak=std::max(peak,takeoff.p.y);}bool canTakeoff=peak>2.5f;
  Replay replayLoop;Frame first{},second{};first.n=11;second.n=22;replayLoop.frames={first,second};replayLoop.playing=true;bool loops=replayLoop.next()->n==11&&replayLoop.next()->n==22&&replayLoop.next()->n==11;
  bool ok=deterministic&&finite&&canTakeoff&&loops;std::ofstream("self-test.txt")<<(ok?"PASS":"FAIL")<<" deterministic_hash="<<std::hex<<hashA<<std::dec<<" frames=2400 finite="<<finite<<" repeatable="<<deterministic<<" takeoff_peak_m="<<peak<<" can_takeoff="<<canTakeoff<<" replay_loops="<<loops<<"\n";return ok?0:1;
}

int WINAPI wWinMain(HINSTANCE h,HINSTANCE,PWSTR cmd,int){if(wcsstr(cmd,L"--self-test"))return selfTest();App a;if(!a.init(h))return 2;MSG m{};while(m.message!=WM_QUIT){while(PeekMessageW(&m,nullptr,0,0,PM_REMOVE)){TranslateMessage(&m);DispatchMessageW(&m);}a.tick();Sleep(1);}return 0;}
