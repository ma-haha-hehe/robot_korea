%module o3py

#pragma SWIG nowarn=325,503,520

%{
#define SWIG_FILE_WITH_INIT

#include <sstream>
#include "o3p/Definitions.hpp"
#include "o3p/o3p.hpp"
#include "o3p/Common.hpp"
#include "o3p/Context.hpp"
#include "o3p/Device.hpp"
#include "o3p/FrameSet.hpp"
#include "o3p/Stream.hpp"
#include "o3p/PipelineConfig.hpp"
#include "o3p/PipelineProfile.hpp"
#include "o3p/Pipeline.hpp"
#include "o3p/info/InfoInterface.hpp"
#include "o3p/info/InfoContainer.hpp"
#include "o3p/option/OptionDescription.hpp"
#include "o3p/option/OptionValue.hpp"
#include "o3p/option/OptionRange.hpp"
#include "o3p/option/OptionInfo.hpp"
#include "o3p/option/Option.hpp"
#include "o3p/option/OptionsInterface.hpp"

using namespace o3p;
%}

%include <std_except.i>

 // Declare the exception handler
%exception {
    try {
        $action
    } catch (const BagFileEndReachedWarning &e) {
        PyErr_SetString(PyExc_EOFError, e.what());
        SWIG_fail;
    } catch (const UsbDisconnectedException &e) {
        PyErr_SetString(PyExc_ConnectionError, e.what());
        SWIG_fail;
    } catch (const std::exception& e) {
        PyErr_SetString(PyExc_RuntimeError, e.what());
        SWIG_fail;
    }
}

%ignore o3p::Frame::operator=(const Frame &other);
%ignore o3p::FrameSet::operator=(const FrameSet &other);
%ignore o3p::VideoFrame::operator=(const VideoFrame &other);
%ignore o3p::DepthFrame::operator=(const DepthFrame &other);
%ignore o3p::PointcloudFrame::operator=(const PointcloudFrame &other);

%include "std_shared_ptr.i"
%include "std_vector.i"
%include "std_pair.i"
%include <stdint.i>
%include <std_string.i>

%include "numpy.i"  

%init %{
import_array();
%}

%template(VectorStream) std::vector<std::shared_ptr<o3p::Stream>>;
%template(VectorString) std::vector<std::string>;
%template(VectorFloat) std::vector<float>;
%template(VectorVectorFloat) std::vector<std::vector<float>>;
%template(VectorUint16t) std::vector<uint16_t>;
%template(VectorUint8t) std::vector<uint8_t>;

%typemap(in) (const std::vector<uint8_t> &) {
    PyArrayObject *array = NULL;

    if (!PyArray_Check($input)) {
        SWIG_exception_fail(SWIG_TypeError, "Expected a NumPy array");
    }

    array = (PyArrayObject*)$input;

    if (PyArray_TYPE(array) != NPY_UINT8) {
        SWIG_exception_fail(SWIG_TypeError, "Expected a NumPy array of dtype=uint8");
    }

    if (!PyArray_ISCONTIGUOUS(array)) {
        SWIG_exception_fail(SWIG_ValueError, "NumPy array must be contiguous");
    }

    npy_intp size = PyArray_SIZE(array);
    uint8_t *data = static_cast<uint8_t*>(PyArray_DATA(array));

    $1 = new std::vector<uint8_t>(data, data + size);
}

%typemap(freearg) (const std::vector<uint8_t> &) {
    delete $1;
}

%typemap(out) std::vector<uint8_t> {
    npy_intp dims[1] = {(npy_intp)$1.size()};
    PyObject *out = PyArray_SimpleNew(1, dims, NPY_UINT8);
    if (!out) SWIG_fail;

    memcpy(PyArray_DATA((PyArrayObject*)out), $1.data(), $1.size() * sizeof(uint8_t));
    $result = out;
}

%typemap(out) std::vector<uint16_t> {
    npy_intp dims[1] = {(npy_intp)$1.size()};
    PyObject *out = PyArray_SimpleNew(1, dims, NPY_UINT16);
    if (!out) SWIG_fail;

    memcpy(PyArray_DATA((PyArrayObject*)out), $1.data(), $1.size() * sizeof(uint16_t));
    $result = out;
}

%shared_ptr(o3p::Stream);
%shared_ptr(o3p::VideoStream);
%shared_ptr(o3p::Device);
%shared_ptr(o3p::Option);
%shared_ptr(o3p::OptionValueRange);
%shared_ptr(o3p::IntOptionRange);
%shared_ptr(o3p::FloatOptionRange);
%shared_ptr(o3p::StringOptionRange);

%extend o3p::VideoStream {
    static std::shared_ptr<o3p::VideoStream> fromStream(const std::shared_ptr<o3p::Stream>& stream) {
        return std::static_pointer_cast<o3p::VideoStream>(stream);
    }
}

%extend o3p::DepthFrame {
    std::vector<uint16_t> getData() {
        return std::vector<uint16_t>(self->m_data, self->m_data + (self->getWidth() * self->getHeight()));
    }
}

%extend o3p::VideoFrame {
    std::vector<uint8_t> getData() {
        return std::vector<uint8_t>(self->m_data, self->m_data + ((self->getWidth() * self->getHeight() * self->m_bitsPerPixel) / 8));
    }
}

%extend o3p::PluginFrame {
    std::vector<uint8_t> getPluginData() {
        return std::vector<uint8_t>(self->m_data, self->m_data + self->getSize());
    }
}

%extend o3p::ImuFrame {
    std::vector<float> getAcceleration() {
        return std::vector<float>(self->acceleration, self->acceleration + 3);
    }

    std::vector<float> getAngularVelocity() {
        return std::vector<float>(self->angularVelocity, self->angularVelocity + 3);
    }
}

%extend o3p::Intrinsics {
    std::vector<float> getCoeffs() {
        return std::vector<float>(self->coeffs, self->coeffs + 5);
    }
}

%extend o3p::Extrinsics {
    PyObject* getRotation() {
        npy_intp dims[2] = {3, 3};
        PyObject *out = PyArray_SimpleNew(2, dims, NPY_FLOAT32);
        if (!out) return nullptr;

        float *dst = static_cast<float*>(PyArray_DATA((PyArrayObject*)out));
        // Transpose from column-major (storage) to row-major (NumPy C-order)
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                dst[row * 3 + col] = self->rotation[col * 3 + row];
            }
        }
        return out;
    }

    std::vector<float> getTranslation() {
        return std::vector<float>(self->translation, self->translation + 3);
    }
}

%extend o3p::PointcloudFrame {
    PyObject* getData() {
        auto w = self->getWidth();
        auto h = self->getHeight();
        npy_intp dims[3] = {(npy_intp)h, (npy_intp)w, 4};
        PyObject *out = PyArray_SimpleNew(3, dims, NPY_FLOAT32);
        if (!out) return nullptr;

        memcpy(PyArray_DATA((PyArrayObject*)out), self->m_data, h * w * 4 * sizeof(float));
        return out;
    }
}

%extend o3p::PipelineProfile{
    PipelineProfile(const std::vector<std::shared_ptr<o3p::Stream>>& streams, const std::string& serial) {
        return new o3p::PipelineProfile(nullptr, streams, serial);
    }

    std::string getDeviceInfo(o3p::CameraInfo infoType) {
        if (self->getDevice() && self->getDevice()->supportsInfo(infoType)) {
            return self->getDevice()->getInfo(infoType).c_str();
        } else {
            throw std::runtime_error("Device does not support the requested CameraInfo type.");
        }
    }
}

// Ignore C++ cast operators on OptionValue (use asInt/asFloat/asBool/asString instead)
%ignore o3p::OptionValue::operator int;
%ignore o3p::OptionValue::operator float;
%ignore o3p::OptionValue::operator bool;
%ignore o3p::OptionValue::operator std::string;

// Ignore internal/protected members not needed in Python
%ignore o3p::OptionObserver;
%ignore o3p::OptionCommand;
%ignore o3p::OptionCommandType;
%ignore o3p::optionCommandTypeToString;
%ignore o3p::Option::registerObserver;
%ignore o3p::Option::unregisterObserver;
%ignore o3p::OptionValue::serialize;
%ignore o3p::OptionValue::deserialize;
%ignore o3p::OptionValueRange::serialize;
%ignore o3p::IntOptionRange::serialize;
%ignore o3p::IntOptionRange::deserialize;
%ignore o3p::FloatOptionRange::serialize;
%ignore o3p::FloatOptionRange::deserialize;
%ignore o3p::StringOptionRange::serialize;
%ignore o3p::StringOptionRange::deserialize;
%ignore o3p::deserializeOptionValueRange;

%extend o3p::OptionValue {
    int asInt() const { return static_cast<int>(*$self); }
    float asFloat() const { return static_cast<float>(*$self); }
    bool asBool() const { return static_cast<bool>(*$self); }
    std::string asString() const { return static_cast<std::string>(*$self); }

    std::string __str__() {
        std::ostringstream os;
        os << *$self;
        return os.str();
    }
}

%extend o3p::OptionValueRange {
    std::string __str__() {
        std::ostringstream os;
        os << *$self;
        return os.str();
    }
}

%extend o3p::IntOptionRange {
    static std::shared_ptr<o3p::IntOptionRange> fromRange(const std::shared_ptr<o3p::OptionValueRange>& range) {
        return std::dynamic_pointer_cast<o3p::IntOptionRange>(range);
    }
}

%extend o3p::FloatOptionRange {
    static std::shared_ptr<o3p::FloatOptionRange> fromRange(const std::shared_ptr<o3p::OptionValueRange>& range) {
        return std::dynamic_pointer_cast<o3p::FloatOptionRange>(range);
    }
}

%extend o3p::StringOptionRange {
    static std::shared_ptr<o3p::StringOptionRange> fromRange(const std::shared_ptr<o3p::OptionValueRange>& range) {
        return std::dynamic_pointer_cast<o3p::StringOptionRange>(range);
    }
}

%nodefaultctor UsbDisconnectedException;
%nodefaultdtor BagFileEndReachedWarning;

%include "o3p/Definitions.hpp"
%include "o3p/o3p.hpp"
%include "o3p/LensModelType.hpp"
%include "o3p/Common.hpp"
%include "o3p/Context.hpp"
%include "o3p/info/InfoInterface.hpp"
%include "o3p/info/InfoContainer.hpp"
%include "o3p/option/OptionDescription.hpp"
%include "o3p/option/OptionValue.hpp"
%include "o3p/option/OptionRange.hpp"
%include "o3p/option/OptionInfo.hpp"
%include "o3p/option/Option.hpp"
%include "o3p/option/OptionsInterface.hpp"
%include "o3p/Device.hpp"
%include "o3p/FrameSet.hpp"
%include "o3p/Stream.hpp"
%include "o3p/PipelineConfig.hpp"
%include "o3p/PipelineProfile.hpp"
%include "o3p/Pipeline.hpp"

%extend o3p::Frame {
    %template(getMetadataUint32) o3p::Frame::getMetadata<uint32_t>;
    %template(getMetadataUint64) o3p::Frame::getMetadata<uint64_t>;
    %template(getMetadataFloat) o3p::Frame::getMetadata<float>;
    %template(getMetadataBool) o3p::Frame::getMetadata<bool>;
}