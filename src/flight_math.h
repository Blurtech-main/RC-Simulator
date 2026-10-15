#pragma once
#include <algorithm>
#include <cmath>

inline constexpr float PI = 3.14159265358979323846f;

struct V3 { float x{}, y{}, z{}; };
inline V3 operator+(V3 a,V3 b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
inline V3 operator-(V3 a,V3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
inline V3 operator-(V3 a){return {-a.x,-a.y,-a.z};}
inline V3 operator*(V3 a,float s){return {a.x*s,a.y*s,a.z*s};}
inline V3 operator*(float s,V3 a){return a*s;}
inline V3 operator/(V3 a,float s){return a*(1.0f/s);}
inline V3& operator+=(V3& a,V3 b){a=a+b;return a;}
inline V3& operator-=(V3& a,V3 b){a=a-b;return a;}
inline float dot(V3 a,V3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
inline V3 cross(V3 a,V3 b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
inline float len2(V3 a){return dot(a,a);}
inline float len(V3 a){return std::sqrt(len2(a));}
inline V3 norm(V3 a){float l=len(a);return l>1e-6f?a/l:V3{};}
inline float clampf(float v,float a,float b){return std::clamp(v,a,b);}

struct Quat { float w{1},x{},y{},z{}; };
inline Quat qmul(Quat a,Quat b){return {a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w};}
inline Quat qnorm(Quat q){float l=std::sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z);return l>1e-8f?Quat{q.w/l,q.x/l,q.y/l,q.z/l}:Quat{};}
inline V3 rotate(Quat q,V3 v){Quat r=qmul(qmul(q,{0,v.x,v.y,v.z}),{q.w,-q.x,-q.y,-q.z});return {r.x,r.y,r.z};}
inline V3 invrot(Quat q,V3 v){return rotate({q.w,-q.x,-q.y,-q.z},v);}

struct Controls { float ail{},ele{},thr{},rud{}; };
