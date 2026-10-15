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
#include "aircraft_model.h"
#include "flight_dynamics.h"
#include "textured_aircraft_renderer.h"

constexpr float DT = 1.0f / 120.0f;
constexpr int AXES = 8;
constexpr int CHANNELS = 4;
constexpr int ID_RECORD_FLIGHT = 1001;
constexpr int ID_STOP_FLIGHT = 1002;
constexpr int ID_CLEAR_FLIGHT = 1003;
constexpr int ID_MANUAL_OVERRIDE = 1004;

static void recenterWheelElevator(float& value,float& holdSeconds,float dt){
  if(holdSeconds>0.f){holdSeconds=std::max(0.f,holdSeconds-dt);return;}
  constexpr float returnRate=1.25f;
  float step=returnRate*dt;
  if(std::abs(value)<=step)value=0.f;
  else value-=std::copysign(step,value);
}

struct AxisCal { long lo{-1000}, hi{1000}, center{}; float dead{0.035f}; };
struct Mapping { std::array<int,CHANNELS> axis{0,1,2,5}; std::array<bool,CHANNELS> reverse{false,true,false,false}; };

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
  V3 p{0,.20f,0},v{},omega{}; Quat q{}; bool crashed{},hasFlown{};
  float aoa{},beta{},airspeed{},lift{},drag{},thrust{},reynolds{},stall{},propRpm{},loadFactor{};
  void reset(){*this=Plane{};}
  void step(Controls c,float dt,const AtmosphereSample& atmosphere={}){
    if(crashed)return;
    const auto& aircraft=TrainerAircraftModel::instance();
    FlightState state{p,v,omega,q,propRpm};
    auto aero=evaluateAerodynamics(state,c,atmosphere);
    bool onGear=p.y+aircraft.lowestCollisionY(q.w,q.x,q.y,q.z)<=.003f;
    if(onGear)aero.momentBody.z+=aero.telemetry.propTorque*TrainerPropRollCoupling; // tyre reaction carries prop torque on the roll
    integrateFlightState(state,aero,dt);
    p=state.positionWorld;v=state.velocityWorld;omega=state.omegaBody;q=state.attitude;propRpm=state.propRpm;
    aoa=aero.telemetry.aoa;beta=aero.telemetry.beta;airspeed=aero.telemetry.airspeed;
    lift=aero.telemetry.lift;drag=aero.telemetry.drag;thrust=aero.telemetry.thrust;
    reynolds=aero.telemetry.reynolds;stall=aero.telemetry.stallFraction;loadFactor=aero.telemetry.loadFactor;
    float contactY=p.y+aircraft.lowestCollisionY(q.w,q.x,q.y,q.z);
    if(contactY>.08f)hasFlown=true;
    if(contactY<0.f){
      p.y-=contactY;
      float upDot=rotate(q,{0,1,0}).y,impact=std::max(0.f,-v.y);
      bool gearAttitude=upDot>.72f&&std::abs(rotate(q,{1,0,0}).y)<.42f;
      if((hasFlown&&impact>4.5f)||!gearAttitude)crashed=true;
      v.y=std::max(0.f,-v.y*.08f);v.x*=.9985f;v.z*=.9985f;
      // The three gear contacts provide a broad support polygon. Resolve its
      // angular impulse here so propeller reaction torque cannot tip a parked
      // aircraft through the runway before aerodynamic controls have authority.
      omega.x*=.995f;omega.y*=.94f;omega.z*=.90f;
    }
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
      case LAND:if(p.p.y<=TrainerAircraftModel::instance().groundClearance()+.01f&&p.airspeed<10.f){score=std::max(0,score-int(elapsed*2));stage=COMPLETE;}break;
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
  HWND wnd{},recordButton{},stopButton{},clearButton{},overrideCheck{};HINSTANCE inst{};Controller ctrl;AudioPpm ppm;Plane plane;Replay replay;Lesson lesson;Controls liveControls{};bool help{};bool keyLeft{},keyRight{},keyUp{},keyDown{},keyW{},keyS{},keyA{},keyD{};int selected{};uint64_t frame{};double accum{},paintAccum{},crashResetElapsed{};LARGE_INTEGER last{},freq{};
  static App* self(HWND w){return reinterpret_cast<App*>(GetWindowLongPtrW(w,GWLP_USERDATA));}
  static LRESULT CALLBACK proc(HWND w,UINT m,WPARAM wp,LPARAM lp){
    if(m==WM_NCCREATE)SetWindowLongPtrW(w,GWLP_USERDATA,(LONG_PTR)((CREATESTRUCT*)lp)->lpCreateParams);App* a=self(w);if(!a)return DefWindowProcW(w,m,wp,lp);
    if(m==WM_DESTROY){PostQuitMessage(0);return 0;}if(m==WM_PAINT){a->paint();return 0;}if(m==WM_KEYDOWN){a->setFlightKey((int)wp,true);if((lp&(1ll<<30))==0)a->key((int)wp);return 0;}if(m==WM_KEYUP){a->setFlightKey((int)wp,false);return 0;}if(m==WM_KILLFOCUS){a->clearFlightKeys();return 0;}if(m==WM_MOUSEWHEEL){a->scrollRightStick(GET_WHEEL_DELTA_WPARAM(wp));return 0;}if(m==WM_COMMAND){a->command(LOWORD(wp),HIWORD(wp));return 0;}return DefWindowProcW(w,m,wp,lp);
  }
  bool init(HINSTANCE h){inst=h;QueryPerformanceFrequency(&freq);QueryPerformanceCounter(&last);WNDCLASSW c{};c.lpfnWndProc=proc;c.hInstance=h;c.lpszClassName=L"RCTrainerSlice";c.hCursor=LoadCursor(nullptr,IDC_ARROW);c.hbrBackground=(HBRUSH)GetStockObject(BLACK_BRUSH);RegisterClassW(&c);wnd=CreateWindowW(c.lpszClassName,L"RC Trainer - DX6 to flying trainer",WS_OVERLAPPEDWINDOW|WS_VISIBLE,CW_USEDEFAULT,CW_USEDEFAULT,1280,800,nullptr,nullptr,h,this);if(!wnd)return false;
    recordButton=CreateWindowW(L"BUTTON",L"RECORD FLIGHT",WS_CHILD|WS_VISIBLE|BS_PUSHBUTTON,20,260,135,28,wnd,(HMENU)(INT_PTR)ID_RECORD_FLIGHT,h,nullptr);
    stopButton=CreateWindowW(L"BUTTON",L"STOP",WS_CHILD|WS_VISIBLE|BS_PUSHBUTTON,165,260,80,28,wnd,(HMENU)(INT_PTR)ID_STOP_FLIGHT,h,nullptr);
    clearButton=CreateWindowW(L"BUTTON",L"CLEAR",WS_CHILD|WS_VISIBLE|BS_PUSHBUTTON,255,260,80,28,wnd,(HMENU)(INT_PTR)ID_CLEAR_FLIGHT,h,nullptr);
    overrideCheck=CreateWindowW(L"BUTTON",L"MANUAL OVERRIDE",WS_CHILD|WS_VISIBLE|BS_AUTOCHECKBOX,345,260,170,28,wnd,(HMENU)(INT_PTR)ID_MANUAL_OVERRIDE,h,nullptr);
    // Never auto-play an old flight on launch.  A saved replay may end in a
    // crash, which made a fresh session appear to crash instantly.  F6 still
    // loads it explicitly when the pilot actually asks for playback.
    ctrl.init(h,wnd);ctrl.load();bool audio=ppm.init();const auto& aircraft=TrainerAircraftModel::instance();const auto& visual=TexturedAircraftRenderer::instance();std::ofstream diagnostic("controller-diagnostic.txt");diagnostic<<"AUDIO_PPM_CAPTURE="<<(audio?"OPEN":"FAILED")<<"\nDIRECTINPUT="<<(ctrl.name==L"Keyboard fallback"?"NONE":"FOUND")<<"\nGROUND_RENDERER=PROCEDURAL\nAIRCRAFT_MODEL=CESSNA_172_CC_BY_4_0\nTEXTURED_ASSET="<<(visual.available()?"LOADED":"FAILED")<<"\nCOLLISION_PRIMITIVES="<<aircraft.collision().size()<<"\nAERO_REFERENCE_SURFACES="<<aircraft.aeroSurfaces().size()<<"\n";return true;}
  float keyboardThrottle{},keyboardElevator{},keyboardAileron{},wheelElevatorHold{};
  void setFlightKey(int k,bool down){if(k==VK_LEFT)keyLeft=down;else if(k==VK_RIGHT)keyRight=down;else if(k==VK_UP)keyUp=down;else if(k==VK_DOWN)keyDown=down;else if(k=='W')keyW=down;else if(k=='S')keyS=down;else if(k=='A')keyA=down;else if(k=='D')keyD=down;}
  void clearFlightKeys(){keyLeft=keyRight=keyUp=keyDown=keyW=keyS=keyA=keyD=false;}
  Controls keyboard(){
    // Flight keys come only from messages delivered to this window.  Global
    // polling made ordinary typing in other apps fly and crash the plane.
    Controls c{};
    // Keys are digital, so make an arrow a modest, eased trainer input rather
    // than the same full-deflection aileron command as a transmitter stick.
    float targetAileron=.35f*float(keyRight-keyLeft);
    keyboardAileron+=clampf(DT*8.f,0.f,1.f)*(targetAileron-keyboardAileron);
    // Down arrow is physically "pull the stick back" and therefore nose-up.
    // A mouse wheel has no visible stick position, so its elevator pulse must
    // spring back to centre. Persisting it forever made a short scroll hold
    // full elevator, stall the aircraft and look like an invisible flight wall.
    c.ail=keyboardAileron;c.ele=clampf(keyboardElevator+.55f*float(keyDown-keyUp),-1.f,1.f);c.thr=keyboardThrottle;c.rud=float(keyD-keyA);
    keyboardThrottle=clampf(keyboardThrottle+float(keyW-keyS)*DT*.45f,0,1);
    recenterWheelElevator(keyboardElevator,wheelElevatorHold,DT);
    return c;
  }
  void scrollRightStick(short delta){if(GetForegroundWindow()==wnd){keyboardElevator=clampf(keyboardElevator+float(delta)/WHEEL_DELTA*.1f,-1.f,1.f);wheelElevatorHold=.22f;}}
  void startRecording(){replay.startRecord();frame=0;SendMessageW(overrideCheck,BM_SETCHECK,BST_UNCHECKED,0);}
  void stopFlight(){replay.stop();}
  void resetFlight(){plane.reset();lesson.reset();frame=0;crashResetElapsed=0;keyboardThrottle=0;keyboardElevator=0;keyboardAileron=0;wheelElevatorHold=0;clearFlightKeys();}
  void command(int id,int event){if(event!=BN_CLICKED)return;if(id==ID_RECORD_FLIGHT)startRecording();else if(id==ID_STOP_FLIGHT)stopFlight();else if(id==ID_CLEAR_FLIGHT)replay.clear();else if(id==ID_MANUAL_OVERRIDE&&SendMessageW(overrideCheck,BM_GETCHECK,0,0)==BST_CHECKED)replay.playing=false;}
  void key(int k){
    if(k==VK_ESCAPE)DestroyWindow(wnd);else if(k==VK_F1)help=!help;else if(k==VK_F2){ppm.nextSource();}else if(k=='C'){if(ctrl.calibrating)ctrl.endCal();else ctrl.beginCal();}
    else if(k==VK_RETURN&&ctrl.calibrating)ctrl.endCal();else if(k>='1'&&k<='4')selected=k-'1';
    else if(k==VK_OEM_4){ctrl.map.axis[selected]=(ctrl.map.axis[selected]+AXES-1)%AXES;ctrl.save();}
    else if(k==VK_OEM_6){ctrl.map.axis[selected]=(ctrl.map.axis[selected]+1)%AXES;ctrl.save();}
    else if(k=='R'){ctrl.map.reverse[selected]=!ctrl.map.reverse[selected];ctrl.save();}
    else if(k==VK_OEM_MINUS||k==VK_OEM_PLUS){auto& d=ctrl.cal[ctrl.map.axis[selected]].dead;d=clampf(d+(k==VK_OEM_PLUS?.005f:-.005f),0,.25f);ctrl.save();}
    else if(k==VK_F3){resetFlight();}else if(k==VK_F5){if(replay.recording)replay.stopRecord();else startRecording();}else if(k==VK_F6)replay.load();else if(k==VK_F7)stopFlight();else if(k==VK_SPACE){resetFlight();}
  }
  void tick(){
    LARGE_INTEGER now;QueryPerformanceCounter(&now);double elapsed=double(now.QuadPart-last.QuadPart)/freq.QuadPart;last=now;elapsed=std::min(elapsed,.1);accum+=elapsed;paintAccum+=elapsed;ctrl.poll();
    while(accum>=DT){Controls c{};if(replay.playing){if(const Frame* saved=replay.next()){c=saved->c;plane=saved->p;}}else{c=ppm.active()?ppm.controls():(ctrl.name==L"Keyboard fallback"?keyboard():ctrl.controls());plane.step(c,DT);lesson.update(plane,c,DT);if(plane.crashed){crashResetElapsed+=DT;if(crashResetElapsed>=2.0)resetFlight();}else crashResetElapsed=0;replay.write(frame,c,plane);}liveControls=c;frame++;accum-=DT;}if(paintAccum>=1.0/60.0){paintAccum=std::fmod(paintAccum,1.0/60.0);InvalidateRect(wnd,nullptr,FALSE);}
  }
  static void txt(HDC dc,int x,int y,const std::wstring&s,COLORREF col=RGB(235,240,240)){SetTextColor(dc,col);SetBkMode(dc,TRANSPARENT);TextOutW(dc,x,y,s.c_str(),(int)s.size());}
  static std::wstring f1(float v){wchar_t b[32];swprintf_s(b,L"%.1f",v);return b;}
  void paint(){
    PAINTSTRUCT ps;HDC dc=BeginPaint(wnd,&ps);RECT rc;GetClientRect(wnd,&rc);HDC mem=CreateCompatibleDC(dc);HBITMAP bm=CreateCompatibleBitmap(dc,rc.right,rc.bottom);auto old=SelectObject(mem,bm);
    HBRUSH sky=CreateSolidBrush(RGB(94,157,207));FillRect(mem,&rc,sky);DeleteObject(sky);
    // A quaternion's individual components may change sign without changing its
    // orientation.  Use the aircraft's actual forward direction for a stable,
    // camera-like horizon, and use altitude in every ground-plane projection.
    V3 cameraForward=rotate(plane.q,{0,0,1});float cameraHeight=std::max(TrainerAircraftModel::instance().groundClearance(),plane.p.y);
    // Keep the aircraft readable against the field: pitch is a small camera
    // cue, while altitude is what makes the ground recede.  A large pitch
    // camera shift can push the field off-screen and make ground contact look
    // like an unexplained mid-air crash.
    int horizon=int(rc.bottom*.48f+clampf(cameraForward.y,-.7f,.7f)*rc.bottom*.10f);
    POINT ground[4]={{0,horizon},{rc.right,horizon},{rc.right,rc.bottom},{0,rc.bottom}};HBRUSH grassFill=CreateSolidBrush(RGB(65,111,57));auto ob=SelectObject(mem,grassFill);Polygon(mem,ground,4);SelectObject(mem,ob);DeleteObject(grassFill);
    // Deliberately huge and cheap: the field above is one solid polygon that
    // reaches the horizon forever.  This single runway wedge changes apparent
    // width with height, giving a strong altitude cue for one Polygon call.
    int vanishingX=rc.right/2;
    float groundDepth=float(std::max(1,int(rc.bottom)-horizon));
    float runwayScale=groundDepth/std::max(cameraHeight,1.2f);
    int runwayCentre=vanishingX-int(plane.p.x*runwayScale);
    int runwayHalf=std::clamp(int(9.f*runwayScale),18,int(rc.right)*2);
    POINT runway[4]={{vanishingX-2,horizon},{vanishingX+2,horizon},{runwayCentre+runwayHalf,rc.bottom},{runwayCentre-runwayHalf,rc.bottom}};
    HBRUSH runwayFill=CreateSolidBrush(RGB(91,103,74));ob=SelectObject(mem,runwayFill);Polygon(mem,runway,4);SelectObject(mem,ob);DeleteObject(runwayFill);

    // Sixteen near-spaced cross marks replace the old ~90,000 grass segments.
    // At runway height the nearest mark is still visibly separated from the
    // horizon; during flight their perspective spread makes height obvious.
    HPEN scaleMarks=CreatePen(PS_SOLID,1,RGB(78,128,63));auto op=SelectObject(mem,scaleMarks);
    float travel=std::fmod(std::max(0.f,plane.p.z),8.f);
    for(int i=1;i<=16;i++){
      float d=i*8.f-travel;if(d<.75f)d+=128.f;
      int y=horizon+int(900.f*cameraHeight/d);
      if(y>horizon&&y<rc.bottom){MoveToEx(mem,0,y,nullptr);LineTo(mem,rc.right,y);}
    }
    // Two edges make lateral drift visible without drawing a dense grid.
    MoveToEx(mem,vanishingX,horizon,nullptr);LineTo(mem,-rc.right/3,rc.bottom);
    MoveToEx(mem,vanishingX,horizon,nullptr);LineTo(mem,rc.right+rc.right/3,rc.bottom);
    SelectObject(mem,op);DeleteObject(scaleMarks);
    int cx=rc.right/2;float agl=std::max(0.f,plane.p.y-TrainerAircraftModel::instance().groundClearance());int cy=horizon+10-int(std::min(105.f,agl*5.25f));
    // The old screen-space silhouette only used bank, so elevator could move
    // the simulated aircraft without ever moving its visible nose.  Pitch is
    // the vertical component of its real forward vector: make that explicit
    // in the fuselage and tail geometry, rather than using an Euler component
    // (which would be unstable once the trainer has banked).
    float visualPitch=std::asin(clampf(cameraForward.y,-1.f,1.f));
    float modelScale=210.f*(1.f-.10f*std::abs(visualPitch));
    auto& visual=TexturedAircraftRenderer::instance();visual.draw(mem,cx,cy,modelScale,plane.q.w,plane.q.x,plane.q.y,plane.q.z);
    if(!visual.available())txt(mem,cx-210,cy,L"AIRCRAFT ASSET ERROR: "+visual.error(),RGB(255,90,70));
    HFONT font=CreateFontW(18,0,0,0,FW_NORMAL,FALSE,FALSE,FALSE,DEFAULT_CHARSET,OUT_DEFAULT_PRECIS,CLIP_DEFAULT_PRECIS,CLEARTYPE_QUALITY,FIXED_PITCH,L"Consolas");auto of=SelectObject(mem,font);
    bool ppmLive=ppm.active(),ppmHit=ppm.hit();auto ppmCh=ppm.channels();float mic=ppm.audioLevel();std::wstring input=ppmLive?L"DX6i 3.5mm PPM":ctrl.name;std::wstring mode=replay.playing?L"PLAYBACK":replay.recording?L"RECORDING":L"LIVE";txt(mem,20,18,L"RC TRAINER  |  "+mode,RGB(255,225,80));txt(mem,20,44,L"Control: "+input);txt(mem,20,70,L"Audio source [F2]: "+ppm.sourceName(),RGB(255,225,80));txt(mem,20,96,L"TAS "+f1(plane.airspeed)+L" m/s   Alt AGL "+f1(std::max(0.f,plane.p.y-TrainerAircraftModel::instance().groundClearance()))+L" m   Pitch "+f1(visualPitch*180/PI)+L" deg   AoA "+f1(plane.aoa*180/PI)+L" deg   Beta "+f1(plane.beta*180/PI)+L" deg");txt(mem,20,122,L"L/D/T "+f1(plane.lift)+L" / "+f1(plane.drag)+L" / "+f1(plane.thrust)+L" N   Re "+std::to_wstring(int(plane.reynolds/1000))+L"k   Prop "+std::to_wstring(int(plane.propRpm))+L" rpm   Stall "+std::to_wstring(int(plane.stall*100))+L"%");txt(mem,20,148,L"AUDIO SIGNAL",RGB(255,225,80));Rectangle(mem,155,149,375,165);COLORREF meterCol=ppmLive?RGB(70,230,100):(ppmHit?RGB(255,225,50):RGB(255,160,45));HBRUSH mb=CreateSolidBrush(meterCol);RECT mr{157,151,157+int(clampf(mic,0,1)*216),163};FillRect(mem,&mr,mb);DeleteObject(mb);txt(mem,385,147,ppmLive?L"PPM LOCKED":(ppmHit?L"PPM HIT":L"move sticks"),meterCol);
    txt(mem,20,180,L"FLIGHT SCHOOL  "+std::to_wstring(std::min(int(lesson.stage)+1,5))+L"/5",RGB(255,225,80));txt(mem,20,206,lesson.objective(),lesson.failed?RGB(255,90,70):RGB(245,245,245));txt(mem,20,232,L"AIL "+f1(liveControls.ail)+L"  ELE "+f1(liveControls.ele)+L"  THR "+f1(liveControls.thr)+L"  RUD "+f1(liveControls.rud)+L"   Time "+f1(lesson.elapsed)+L" s");
    if(lesson.stage==Lesson::TAKEOFF_ROLL&&plane.airspeed>11.5f)txt(mem,cx-215,175,L"ROTATE: briefly hold DOWN, then release",RGB(255,225,80));
    if(plane.crashed)txt(mem,cx-135,130,L"CRASH — RESTARTING IN "+f1(std::max(0.f,2.f-float(crashResetElapsed)))+L" s",RGB(255,90,70));
    const wchar_t* names[4]={L"AIL",L"ELE",L"THR",L"RUD"};for(int i=0;i<4;i++){int y=rc.bottom-130+i*25;std::wstring s=(i==selected?L"> ":L"  ")+std::wstring(names[i])+L" axis "+std::to_wstring(ctrl.map.axis[i]+1)+(ctrl.map.reverse[i]?L" REV":L"");txt(mem,20,y,s,i==selected?RGB(255,225,80):RGB(235,240,240));}
    for(int i=0;i<AXES;i++){int x=rc.right-300,y=20+i*24;Rectangle(mem,x,y,x+220,y+15);float av=ppmLive?ppmCh[i]:ctrl.value(i);int v=int((av+1)*.5f*216);HBRUSH bar=CreateSolidBrush(RGB(255,195,45));RECT br{x+2,y+2,x+2+v,y+13};FillRect(mem,&br,bar);DeleteObject(bar);txt(mem,x-72,y-2,(ppmLive?L"PPM":L"AX")+std::to_wstring(i+1)+L" "+f1(av));}
    if(ctrl.calibrating)txt(mem,cx-265,210,L"CALIBRATING: move all sticks fully, centre them, press ENTER",RGB(255,225,80));
    if(help){RECT box{cx-330,300,cx+330,600};HBRUSH bb=CreateSolidBrush(RGB(20,30,35));FillRect(mem,&box,bb);DeleteObject(bb);std::array<std::wstring,10> l={L"AERODYNAMIC SLICE CONTROLS",L"Record/Stop saves the exact flight; Clear removes it",L"F6 loads and loops the saved flight",L"Manual Override stops playback and restores live control",L"F2: choose audio-interface input",L"F3: restart flight-school lesson",L"C: calibrate   ENTER: finish at stick centres",L"SPACE: runway reset   ESC: exit",L"Keyboard: W/S throttle; A/D rudder; LEFT/RIGHT aileron",L"Wheel pulses elevator and recentres; DOWN pulls nose-up"};for(size_t i=0;i<l.size();i++)txt(mem,cx-300,320+(int)i*27,l[i],i==0?RGB(255,225,80):RGB(235,240,240));}
    SelectObject(mem,of);DeleteObject(font);BitBlt(dc,0,0,rc.right,rc.bottom,mem,0,0,SRCCOPY);SelectObject(mem,old);DeleteObject(bm);DeleteDC(mem);EndPaint(wnd,&ps);
  }
};

static int selfTest(){
  Plane a,b;uint64_t hashA=1469598103934665603ull,hashB=hashA;auto run=[](Plane&p,uint64_t& h){for(int i=0;i<2400;i++){Controls c{.18f*std::sin(i*.013f),.08f*std::sin(i*.021f),i<300?i/300.f:.72f,.04f};p.step(c,DT);const unsigned char* q=(const unsigned char*)&p;for(size_t k=0;k<sizeof(p);k++){h^=q[k];h*=1099511628211ull;}}};run(a,hashA);run(b,hashB);bool finite=std::isfinite(a.p.x)&&std::isfinite(a.p.y)&&std::isfinite(a.p.z);bool deterministic=hashA==hashB;
  Plane parked;for(int i=0;i<2400&&!parked.crashed;i++){Controls c{};c.ele=1.f;parked.step(c,DT);}bool parkedStable=!parked.crashed&&std::abs(parked.q.x)<.001f&&std::abs(parked.q.z)<.001f;
  Plane takeoff;float peak=takeoff.p.y;for(int i=0;i<1440&&!takeoff.crashed&&peak<3.f;i++){Controls c{};c.thr=1.f;c.ail=i>420?clampf(2.5f*takeoff.q.z-.02f,-.30f,.30f):0.f;c.ele=i>420?(takeoff.p.y<2.f?.40f:-.10f):0.f;takeoff.step(c,DT);peak=std::max(peak,takeoff.p.y);}bool canTakeoff=peak>2.5f&&!takeoff.crashed;
  Plane liveTakeoff;float livePeak=liveTakeoff.p.y,liveThrottle=0.f;int liveRotateFrames=0;for(int i=0;i<2400&&!liveTakeoff.crashed&&livePeak<3.f;i++){liveThrottle=clampf(liveThrottle+.45f*DT,0.f,1.f);Controls c{};c.thr=liveThrottle;if(liveTakeoff.airspeed>11.5f&&liveRotateFrames<72){c.ele=.55f;++liveRotateFrames;}liveTakeoff.step(c,DT);livePeak=std::max(livePeak,liveTakeoff.p.y);}bool livePathTakeoff=livePeak>2.5f&&!liveTakeoff.crashed;
  Plane handsOffTakeoff;float handsOffThrottle=0.f,handsOffPeak=handsOffTakeoff.p.y,maxHandsOffBank=0.f;int rotateFrames=0;
  for(int i=0;i<2400&&!handsOffTakeoff.crashed&&handsOffPeak<3.f;i++){
    handsOffThrottle=clampf(handsOffThrottle+.45f*DT,0.f,1.f);Controls c{};c.thr=handsOffThrottle;
    if(handsOffTakeoff.airspeed>11.5f&&rotateFrames<72){c.ele=.40f;++rotateFrames;}
    handsOffTakeoff.step(c,DT);handsOffPeak=std::max(handsOffPeak,handsOffTakeoff.p.y);
    V3 right=rotate(handsOffTakeoff.q,{1,0,0}),up=rotate(handsOffTakeoff.q,{0,1,0});
    maxHandsOffBank=std::max(maxHandsOffBank,std::abs(std::atan2(right.y,up.y)*180.f/PI));
  }
  bool handsOffStable=handsOffPeak>2.5f&&!handsOffTakeoff.crashed&&maxHandsOffBank<20.f;

  auto atAlpha=[](float alpha,float speed,Controls c={},AtmosphereSample atm={}){FlightState s{};s.positionWorld={0,30,0};s.velocityWorld={0,-std::sin(alpha)*speed,std::cos(alpha)*speed};s.propRpm=0;return evaluateAerodynamics(s,c,atm);};
  auto pitchMinus5=atAlpha(-5*PI/180.f,15.f),pitchZero=atAlpha(0,15.f),pitchPlus5=atAlpha(5*PI/180.f,15.f),pitchPlus10=atAlpha(10*PI/180.f,15.f);
  bool coefficientFinite=true;for(int d=-45;d<=45;d++){auto e=atAlpha(d*PI/180.f,15.f);coefficientFinite&=std::isfinite(e.telemetry.cl)&&std::isfinite(e.telemetry.cd)&&std::isfinite(e.forceBody.x)&&std::isfinite(e.momentBody.z);}
  auto l10=atAlpha(5*PI/180.f,10.f),l20=atAlpha(5*PI/180.f,20.f);float liftScale=l20.telemetry.lift/std::max(.001f,l10.telemetry.lift),dragScale=l20.telemetry.drag/std::max(.001f,l10.telemetry.drag);bool pressureScaling=liftScale>3.75f&&liftScale<4.25f&&dragScale>3.75f&&dragScale<4.25f;
  auto preStall=naca2412LowRe(11*PI/180.f,100000.f),postStall=naca2412LowRe(25*PI/180.f,100000.f);bool stallShape=preStall.cl>.85f&&preStall.cl<1.1f&&postStall.cl<preStall.cl*.9f&&postStall.cd>preStall.cd*2.f;
  auto neutral=atAlpha(2*PI/180.f,15.f),elevator=atAlpha(2*PI/180.f,15.f,{0,.25f,0,0}),aileron=atAlpha(2*PI/180.f,15.f,{.25f,0,0,0}),rudder=atAlpha(2*PI/180.f,15.f,{0,0,0,.25f});
  bool controlSigns=elevator.momentBody.x<neutral.momentBody.x&&aileron.momentBody.z<neutral.momentBody.z&&rudder.momentBody.y>neutral.momentBody.y;
  AtmosphereSample headwind{};headwind.windWorld={0,0,-10};FlightState still{};still.positionWorld={0,30,0};auto windEval=evaluateAerodynamics(still,{},headwind);bool windRelative=std::abs(windEval.telemetry.airspeed-10.f)<.001f&&windEval.forceBody.z<0.f;
  FlightState shifted{};shifted.positionWorld={0,30,0};shifted.velocityWorld={3,0,15};AtmosphereSample shiftedAir{};shiftedAir.windWorld={3,0,5};auto galilean=evaluateAerodynamics(shifted,{},shiftedAir);auto baseline=atAlpha(0,10.f);bool galileanInvariant=std::abs(galilean.telemetry.airspeed-baseline.telemetry.airspeed)<.001f&&len(galilean.forceBody-baseline.forceBody)<.001f;
  Replay replayLoop;Frame first{},second{};first.n=11;second.n=22;replayLoop.frames={first,second};replayLoop.playing=true;bool loops=replayLoop.next()->n==11&&replayLoop.next()->n==22&&replayLoop.next()->n==11;
  const auto& model=TrainerAircraftModel::instance();float uprightLow=model.lowestCollisionY(1,0,0,0);float invertedLow=model.lowestCollisionY(0,0,0,1);bool modelContract=std::abs(uprightLow+model.groundClearance())<.001f&&invertedLow<-.10f&&model.collision().size()==7&&model.aeroSurfaces().size()==8;
  bool aeroSuite=coefficientFinite&&pressureScaling&&stallShape&&controlSigns&&windRelative&&galileanInvariant&&livePathTakeoff;
  float wheelCommand=1.f,wheelHold=.22f;for(int i=0;i<240;i++)recenterWheelElevator(wheelCommand,wheelHold,DT);bool wheelRecentres=wheelCommand==0.f;
  bool ok=deterministic&&finite&&parkedStable&&canTakeoff&&handsOffStable&&loops&&modelContract&&aeroSuite&&wheelRecentres;std::ofstream("self-test.txt")<<(ok?"PASS":"FAIL")<<" deterministic_hash="<<std::hex<<hashA<<std::dec<<" frames=2400 finite="<<finite<<" repeatable="<<deterministic<<" parked_stable="<<parkedStable<<" takeoff_peak_m="<<peak<<" takeoff_speed="<<takeoff.airspeed<<" takeoff_distance="<<takeoff.p.z<<" live_takeoff="<<livePathTakeoff<<" live_peak_m="<<livePeak<<" live_speed="<<liveTakeoff.airspeed<<" hands_off_stable="<<handsOffStable<<" hands_off_peak_m="<<handsOffPeak<<" hands_off_bank_deg="<<maxHandsOffBank<<" hands_off_crashed="<<handsOffTakeoff.crashed<<" pitch_m5="<<pitchMinus5.momentBody.x<<" pitch_0="<<pitchZero.momentBody.x<<" pitch_p5="<<pitchPlus5.momentBody.x<<" pitch_p10="<<pitchPlus10.momentBody.x<<" takeoff_qx="<<takeoff.q.x<<" takeoff_qz="<<takeoff.q.z<<" takeoff_omega_x="<<takeoff.omega.x<<" can_takeoff="<<canTakeoff<<" takeoff_crashed="<<takeoff.crashed<<" coefficient_finite="<<coefficientFinite<<" pressure_scaling="<<pressureScaling<<" lift_q_ratio="<<liftScale<<" drag_q_ratio="<<dragScale<<" stall_shape="<<stallShape<<" cl11="<<preStall.cl<<" cl25="<<postStall.cl<<" elevator_moment_delta="<<(elevator.momentBody.x-neutral.momentBody.x)<<" control_signs="<<controlSigns<<" wind_relative="<<windRelative<<" galilean="<<galileanInvariant<<" replay_loops="<<loops<<" model_contract="<<modelContract<<" wheel_recentres="<<wheelRecentres<<" upright_low_m="<<uprightLow<<" inverted_low_m="<<invertedLow<<" hitboxes="<<model.collision().size()<<" aero_surfaces="<<model.aeroSurfaces().size()<<"\n";return ok?0:1;
}

int WINAPI wWinMain(HINSTANCE h,HINSTANCE,PWSTR cmd,int){if(wcsstr(cmd,L"--self-test"))return selfTest();App a;if(!a.init(h))return 2;MSG m{};while(m.message!=WM_QUIT){while(PeekMessageW(&m,nullptr,0,0,PM_REMOVE)){TranslateMessage(&m);DispatchMessageW(&m);}a.tick();Sleep(1);}return 0;}
