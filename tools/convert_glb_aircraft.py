"""Convert a textured GLB into RCTrainer's compact software-rendering asset.

The original GLB remains beside the derivative. This tool bakes node transforms,
normalizes the real 11.00 m C172 wingspan to the simulator's 1.72 m replica,
and embeds full-resolution RGBA material maps for a dependency-free runtime.
"""
import argparse, json, math, struct
from io import BytesIO
from pathlib import Path
from PIL import Image

COMPONENT={5121:("B",1),5123:("H",2),5125:("I",4),5126:("f",4)}
COUNT={"SCALAR":1,"VEC2":2,"VEC3":3,"VEC4":4}

def mat_mul(a,b):
    return [sum(a[k*4+r]*b[c*4+k] for k in range(4)) for c in range(4) for r in range(4)]
def transform(m,p,w=1.0):
    return tuple(sum(m[c*4+r]*(p[c] if c<3 else w) for c in range(4)) for r in range(3))
def unit(v):
    n=math.sqrt(sum(x*x for x in v)) or 1.0
    return tuple(x/n for x in v)
def local_matrix(n):
    if "matrix" in n:return n["matrix"]
    t=n.get("translation",[0,0,0]);s=n.get("scale",[1,1,1]);x,y,z,w=n.get("rotation",[0,0,0,1])
    return [(1-2*y*y-2*z*z)*s[0],(2*x*y+2*w*z)*s[0],(2*x*z-2*w*y)*s[0],0,
            (2*x*y-2*w*z)*s[1],(1-2*x*x-2*z*z)*s[1],(2*y*z+2*w*x)*s[1],0,
            (2*x*z+2*w*y)*s[2],(2*y*z-2*w*x)*s[2],(1-2*x*x-2*y*y)*s[2],0,t[0],t[1],t[2],1]

def main():
    ap=argparse.ArgumentParser();ap.add_argument("source");ap.add_argument("output");a=ap.parse_args()
    raw=Path(a.source).read_bytes();magic,version,total=struct.unpack_from("<III",raw,0)
    if magic!=0x46546C67 or version!=2:raise SystemExit("GLB 2.0 required")
    off=12;chunks={}
    while off<total:
        size,kind=struct.unpack_from("<II",raw,off);off+=8;chunks[kind]=raw[off:off+size];off+=size
    doc=json.loads(chunks[0x4E4F534A]);blob=chunks[0x004E4942]
    def accessor(index):
        ac=doc["accessors"][index];view=doc["bufferViews"][ac["bufferView"]];fmt,size=COMPONENT[ac["componentType"]];n=COUNT[ac["type"]]
        stride=view.get("byteStride",size*n);start=view.get("byteOffset",0)+ac.get("byteOffset",0)
        return [struct.unpack_from("<"+fmt*n,blob,start+i*stride) for i in range(ac["count"])]
    # Skip Sketchfab's arbitrary presentation transform at root node 0; retain
    # the FBX-to-glTF axis conversion and every authored child transform.
    identity=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1];instances=[]
    def walk(index,parent,skip=False):
        n=doc["nodes"][index];world=parent if skip else mat_mul(parent,local_matrix(n))
        if "mesh" in n:instances.append((doc["meshes"][n["mesh"]],world))
        for child in n.get("children",[]):walk(child,world)
    for root in doc["scenes"][doc.get("scene",0)]["nodes"]:walk(root,identity,root==0)
    triangles=[];all_positions=[]
    for mesh,matrix in instances:
        for p in mesh["primitives"]:
            attrs=p["attributes"];positions=accessor(attrs["POSITION"]);normals=accessor(attrs["NORMAL"]) if "NORMAL" in attrs else [(0,1,0)]*len(positions);uvs=accessor(attrs["TEXCOORD_0"]) if "TEXCOORD_0" in attrs else [(0,0)]*len(positions);indices=[x[0] for x in accessor(p["indices"])]
            out=[]
            for pos,norm,uv in zip(positions,normals,uvs):
                wp=transform(matrix,pos);wn=unit(transform(matrix,norm,0));out.append((wp,wn,uv));all_positions.append(wp)
            material=p.get("material",0)
            for i in range(0,len(indices),3):triangles.append((out[indices[i]],out[indices[i+1]],out[indices[i+2]],material))
    lo=[min(p[i] for p in all_positions) for i in range(3)];hi=[max(p[i] for p in all_positions) for i in range(3)];dims=[hi[i]-lo[i] for i in range(3)]
    # Largest dimension is wingspan, next is length, smallest is height.
    order=sorted(range(3),key=lambda i:dims[i],reverse=True);wing,length,height=order[0],order[1],order[2]
    centre=[(lo[i]+hi[i])/2 for i in range(3)]
    target=(1.72,2.72/11*1.72,8.28/11*1.72) # X span, Y height, Z length
    scales=(target[0]/dims[wing],target[1]/dims[height],target[2]/dims[length])
    def remap(p,is_normal=False):
        q=[p[i]-(0 if is_normal else centre[i]) for i in range(3)]
        v=(q[wing],q[height],q[length])
        return unit(tuple(v[i]/scales[i] for i in range(3))) if is_normal else tuple(v[i]*scales[i] for i in range(3))
    materials=doc.get("materials",[]);images=doc.get("images",[]);textures=doc.get("textures",[]);material_pixels=[]
    for mat in materials:
        pbr=mat.get("pbrMetallicRoughness",{});tex=pbr.get("baseColorTexture",{}).get("index")
        if tex is None:img=Image.new("RGBA",(4,4),tuple(int(x*255) for x in pbr.get("baseColorFactor",[1,1,1,1])))
        else:
            source=textures[tex]["source"];im=images[source];view=doc["bufferViews"][im["bufferView"]];start=view.get("byteOffset",0);img=Image.open(BytesIO(blob[start:start+view["byteLength"]])).convert("RGBA")
        material_pixels.append((1 if mat.get("alphaMode")=="BLEND" else 0,img.width,img.height,img.tobytes()))
    out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True)
    with out.open("wb") as f:
        f.write(b"RCTMESH1");f.write(struct.pack("<IIIfff",2,len(triangles),len(material_pixels),scales[0],target[0],target[2]))
        for flags,w,h,pixels in material_pixels:f.write(struct.pack("<IIII",flags,w,h,len(pixels)));f.write(pixels)
        for tri in triangles:
            for pos,norm,uv in tri[:3]:f.write(struct.pack("<8f",*remap(pos),*remap(norm,True),uv[0],1-uv[1]))
            f.write(struct.pack("<I",tri[3]))
    print(json.dumps({"triangles":len(triangles),"materials":len(material_pixels),"source_bounds":dims,"axis_order":order,"target_bounds_m":target,"axis_scales":scales,"output_bytes":out.stat().st_size},indent=2))
if __name__=="__main__":main()
