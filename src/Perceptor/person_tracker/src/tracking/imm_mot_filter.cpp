#include <Python.h>
#include "tracking/imm_mot_filter.hpp"
#include <stdexcept>
namespace person_tracker {
namespace {
PyObject * module=nullptr;
void check(bool ok) {if(!ok) {PyErr_Print();throw std::runtime_error("IMM-MOT Python backend failed; see traceback");}}
struct Gil {PyGILState_STATE state;Gil():state(PyGILState_Ensure()){} ~Gil(){PyGILState_Release(state);}};
PyObject * list(const std::vector<double> & v) {auto * p=PyList_New(v.size());for(size_t i=0;i<v.size();++i) PyList_SET_ITEM(p,i,PyFloat_FromDouble(v[i]));return p;}
}
void ImmMotFilter::configure(const std::string & adapter,const std::string & repository,const std::string & dependencies,double position_std,double acceleration_psd,double jerk_psd,double turn_acceleration_psd,double vertical_position_psd) {
 if(module)return;
 if(!Py_IsInitialized()) {Py_Initialize();PyEval_SaveThread();}
 Gil lock;
 auto * path=PySys_GetObject("path");
 for(const auto & s:{dependencies,adapter}) {auto * p=PyUnicode_FromString(s.c_str());PyList_Insert(path,0,p);Py_DECREF(p);}
 module=PyImport_ImportModule("adapter");check(module);
 auto * r=PyObject_CallMethod(module,"configure","s",repository.c_str());check(r);Py_DECREF(r);
 r=PyObject_CallMethod(module,"set_options","d",position_std);check(r);Py_DECREF(r);
 r=PyObject_CallMethod(module,"set_process_noise","dddd",acceleration_psd,jerk_psd,turn_acceleration_psd,vertical_position_psd);check(r);Py_DECREF(r);
}
void ImmMotFilter::unpack(void * result) {
 auto * r=static_cast<PyObject *>(result);check(r);
 char * bytes=nullptr;Py_ssize_t length=0;check(PyBytes_AsStringAndSize(PyTuple_GetItem(r,0),&bytes,&length)==0);snapshot_.assign(bytes,length);
 for(int i=0;i<7;++i)state(i)=PyFloat_AsDouble(PyList_GetItem(PyTuple_GetItem(r,1),i));
 for(int i=0;i<16;++i)covariance(i/4,i%4)=PyFloat_AsDouble(PyList_GetItem(PyTuple_GetItem(r,2),i));
 for(int i=0;i<4;++i)probabilities[i]=PyFloat_AsDouble(PyList_GetItem(PyTuple_GetItem(r,3),i));
 check(!PyErr_Occurred());Py_DECREF(r);
}
void ImmMotFilter::initialize(const std::string & mode,const Eigen::Vector3d & p,const Eigen::Vector3d & size,double stamp) {
 check(module);Gil lock;
 auto * a=list({p.x(),p.y(),p.z()});auto * b=list({size.x(),size.y(),size.z()});
 auto * r=PyObject_CallMethod(module,"create","sOOd",mode.c_str(),a,b,stamp);Py_DECREF(a);Py_DECREF(b);unpack(r);
}
void ImmMotFilter::apply(const std::string & operation,const std::vector<double> & values) {
 Gil lock;auto * blob=PyBytes_FromStringAndSize(snapshot_.data(),snapshot_.size());auto * v=list(values);
 auto * r=PyObject_CallMethod(module,"operate","OsO",blob,operation.c_str(),v);Py_DECREF(blob);Py_DECREF(v);unpack(r);
}
}
