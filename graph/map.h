// **********************************************************************
//
// Copyright (c) 2003-2016 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************
//
// Ice version 3.6.2
//
// <auto-generated>
//
// Generated from file `map.ice'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

#ifndef __map_h__
#define __map_h__

#include <IceUtil/PushDisableWarnings.h>
#include <Ice/ProxyF.h>
#include <Ice/ObjectF.h>
#include <Ice/Exception.h>
#include <Ice/LocalObject.h>
#include <Ice/StreamHelpers.h>
#include <Ice/Proxy.h>
#include <Ice/GCObject.h>
#include <Ice/AsyncResult.h>
#include <Ice/Incoming.h>
#include <IceUtil/ScopedArray.h>
#include <IceUtil/Optional.h>
#include <Ice/StreamF.h>
#include <IceUtil/UndefSysMacros.h>

#ifndef ICE_IGNORE_VERSION
#   if ICE_INT_VERSION / 100 != 306
#       error Ice version mismatch!
#   endif
#   if ICE_INT_VERSION % 100 > 50
#       error Beta header file detected
#   endif
#   if ICE_INT_VERSION % 100 < 2
#       error Ice patch level mismatch!
#   endif
#endif

namespace IceProxy
{

namespace RobotPath
{

class OptimalPath;
void __read(::IceInternal::BasicStream*, ::IceInternal::ProxyHandle< ::IceProxy::RobotPath::OptimalPath>&);
::IceProxy::Ice::Object* upCast(::IceProxy::RobotPath::OptimalPath*);

}

}

namespace RobotPath
{

class OptimalPath;
::Ice::Object* upCast(::RobotPath::OptimalPath*);
typedef ::IceInternal::Handle< ::RobotPath::OptimalPath> OptimalPathPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RobotPath::OptimalPath> OptimalPathPrx;
void __patch(OptimalPathPtr&, const ::Ice::ObjectPtr&);

}

namespace RobotPath
{

struct edgeNode
{
    ::std::string source;
    ::std::string target;
    ::Ice::Double d;
    bool task;
};

typedef ::std::vector< ::RobotPath::edgeNode> edgeArray;

}

namespace Ice
{
template<>
struct StreamableTraits< ::RobotPath::edgeNode>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 11;
    static const bool fixedLength = false;
};

template<class S>
struct StreamWriter< ::RobotPath::edgeNode, S>
{
    static void write(S* __os, const ::RobotPath::edgeNode& v)
    {
        __os->write(v.source);
        __os->write(v.target);
        __os->write(v.d);
        __os->write(v.task);
    }
};

template<class S>
struct StreamReader< ::RobotPath::edgeNode, S>
{
    static void read(S* __is, ::RobotPath::edgeNode& v)
    {
        __is->read(v.source);
        __is->read(v.target);
        __is->read(v.d);
        __is->read(v.task);
    }
};

}

namespace RobotPath
{

class Callback_OptimalPath_calcOptimalPath_Base : virtual public ::IceInternal::CallbackBase { };
typedef ::IceUtil::Handle< Callback_OptimalPath_calcOptimalPath_Base> Callback_OptimalPath_calcOptimalPathPtr;

}

namespace IceProxy
{

namespace RobotPath
{

class OptimalPath : virtual public ::IceProxy::Ice::Object
{
public:

    ::RobotPath::edgeArray calcOptimalPath(const ::std::string& __p_start, const ::RobotPath::edgeArray& __p_edges)
    {
        return calcOptimalPath(__p_start, __p_edges, 0);
    }
    ::RobotPath::edgeArray calcOptimalPath(const ::std::string& __p_start, const ::RobotPath::edgeArray& __p_edges, const ::Ice::Context& __ctx)
    {
        return calcOptimalPath(__p_start, __p_edges, &__ctx);
    }
#ifdef ICE_CPP11
    ::Ice::AsyncResultPtr
    begin_calcOptimalPath(const ::std::string& __p_start, const ::RobotPath::edgeArray& __p_edges, const ::IceInternal::Function<void (const ::RobotPath::edgeArray&)>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception = ::IceInternal::Function<void (const ::Ice::Exception&)>(), const ::IceInternal::Function<void (bool)>& __sent = ::IceInternal::Function<void (bool)>())
    {
        return __begin_calcOptimalPath(__p_start, __p_edges, 0, __response, __exception, __sent);
    }
    ::Ice::AsyncResultPtr
    begin_calcOptimalPath(const ::std::string& __p_start, const ::RobotPath::edgeArray& __p_edges, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __completed, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __sent = ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>())
    {
        return begin_calcOptimalPath(__p_start, __p_edges, 0, ::Ice::newCallback(__completed, __sent), 0);
    }
    ::Ice::AsyncResultPtr
    begin_calcOptimalPath(const ::std::string& __p_start, const ::RobotPath::edgeArray& __p_edges, const ::Ice::Context& __ctx, const ::IceInternal::Function<void (const ::RobotPath::edgeArray&)>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception = ::IceInternal::Function<void (const ::Ice::Exception&)>(), const ::IceInternal::Function<void (bool)>& __sent = ::IceInternal::Function<void (bool)>())
    {
        return __begin_calcOptimalPath(__p_start, __p_edges, &__ctx, __response, __exception, __sent);
    }
    ::Ice::AsyncResultPtr
    begin_calcOptimalPath(const ::std::string& __p_start, const ::RobotPath::edgeArray& __p_edges, const ::Ice::Context& __ctx, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __completed, const ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>& __sent = ::IceInternal::Function<void (const ::Ice::AsyncResultPtr&)>())
    {
        return begin_calcOptimalPath(__p_start, __p_edges, &__ctx, ::Ice::newCallback(__completed, __sent));
    }
    
private:

    ::Ice::AsyncResultPtr __begin_calcOptimalPath(const ::std::string& __p_start, const ::RobotPath::edgeArray& __p_edges, const ::Ice::Context* __ctx, const ::IceInternal::Function<void (const ::RobotPath::edgeArray&)>& __response, const ::IceInternal::Function<void (const ::Ice::Exception&)>& __exception, const ::IceInternal::Function<void (bool)>& __sent);
    
public:
#endif

    ::Ice::AsyncResultPtr begin_calcOptimalPath(const ::std::string& __p_start, const ::RobotPath::edgeArray& __p_edges)
    {
        return begin_calcOptimalPath(__p_start, __p_edges, 0, ::IceInternal::__dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_calcOptimalPath(const ::std::string& __p_start, const ::RobotPath::edgeArray& __p_edges, const ::Ice::Context& __ctx)
    {
        return begin_calcOptimalPath(__p_start, __p_edges, &__ctx, ::IceInternal::__dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_calcOptimalPath(const ::std::string& __p_start, const ::RobotPath::edgeArray& __p_edges, const ::Ice::CallbackPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_calcOptimalPath(__p_start, __p_edges, 0, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_calcOptimalPath(const ::std::string& __p_start, const ::RobotPath::edgeArray& __p_edges, const ::Ice::Context& __ctx, const ::Ice::CallbackPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_calcOptimalPath(__p_start, __p_edges, &__ctx, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_calcOptimalPath(const ::std::string& __p_start, const ::RobotPath::edgeArray& __p_edges, const ::RobotPath::Callback_OptimalPath_calcOptimalPathPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_calcOptimalPath(__p_start, __p_edges, 0, __del, __cookie);
    }

    ::Ice::AsyncResultPtr begin_calcOptimalPath(const ::std::string& __p_start, const ::RobotPath::edgeArray& __p_edges, const ::Ice::Context& __ctx, const ::RobotPath::Callback_OptimalPath_calcOptimalPathPtr& __del, const ::Ice::LocalObjectPtr& __cookie = 0)
    {
        return begin_calcOptimalPath(__p_start, __p_edges, &__ctx, __del, __cookie);
    }

    ::RobotPath::edgeArray end_calcOptimalPath(const ::Ice::AsyncResultPtr&);
    
private:

    ::RobotPath::edgeArray calcOptimalPath(const ::std::string&, const ::RobotPath::edgeArray&, const ::Ice::Context*);
    ::Ice::AsyncResultPtr begin_calcOptimalPath(const ::std::string&, const ::RobotPath::edgeArray&, const ::Ice::Context*, const ::IceInternal::CallbackBasePtr&, const ::Ice::LocalObjectPtr& __cookie = 0);
    
public:
    
    ::IceInternal::ProxyHandle<OptimalPath> ice_context(const ::Ice::Context& __context) const
    {
        return dynamic_cast<OptimalPath*>(::IceProxy::Ice::Object::ice_context(__context).get());
    }
    
    ::IceInternal::ProxyHandle<OptimalPath> ice_adapterId(const ::std::string& __id) const
    {
        return dynamic_cast<OptimalPath*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    }
    
    ::IceInternal::ProxyHandle<OptimalPath> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
        return dynamic_cast<OptimalPath*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    }
    
    ::IceInternal::ProxyHandle<OptimalPath> ice_locatorCacheTimeout(int __timeout) const
    {
        return dynamic_cast<OptimalPath*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    }
    
    ::IceInternal::ProxyHandle<OptimalPath> ice_connectionCached(bool __cached) const
    {
        return dynamic_cast<OptimalPath*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    }
    
    ::IceInternal::ProxyHandle<OptimalPath> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
        return dynamic_cast<OptimalPath*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    }
    
    ::IceInternal::ProxyHandle<OptimalPath> ice_secure(bool __secure) const
    {
        return dynamic_cast<OptimalPath*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    }
    
    ::IceInternal::ProxyHandle<OptimalPath> ice_preferSecure(bool __preferSecure) const
    {
        return dynamic_cast<OptimalPath*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    }
    
    ::IceInternal::ProxyHandle<OptimalPath> ice_router(const ::Ice::RouterPrx& __router) const
    {
        return dynamic_cast<OptimalPath*>(::IceProxy::Ice::Object::ice_router(__router).get());
    }
    
    ::IceInternal::ProxyHandle<OptimalPath> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
        return dynamic_cast<OptimalPath*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    }
    
    ::IceInternal::ProxyHandle<OptimalPath> ice_collocationOptimized(bool __co) const
    {
        return dynamic_cast<OptimalPath*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    }
    
    ::IceInternal::ProxyHandle<OptimalPath> ice_invocationTimeout(int __timeout) const
    {
        return dynamic_cast<OptimalPath*>(::IceProxy::Ice::Object::ice_invocationTimeout(__timeout).get());
    }
    
    ::IceInternal::ProxyHandle<OptimalPath> ice_twoway() const
    {
        return dynamic_cast<OptimalPath*>(::IceProxy::Ice::Object::ice_twoway().get());
    }
    
    ::IceInternal::ProxyHandle<OptimalPath> ice_oneway() const
    {
        return dynamic_cast<OptimalPath*>(::IceProxy::Ice::Object::ice_oneway().get());
    }
    
    ::IceInternal::ProxyHandle<OptimalPath> ice_batchOneway() const
    {
        return dynamic_cast<OptimalPath*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    }
    
    ::IceInternal::ProxyHandle<OptimalPath> ice_datagram() const
    {
        return dynamic_cast<OptimalPath*>(::IceProxy::Ice::Object::ice_datagram().get());
    }
    
    ::IceInternal::ProxyHandle<OptimalPath> ice_batchDatagram() const
    {
        return dynamic_cast<OptimalPath*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    }
    
    ::IceInternal::ProxyHandle<OptimalPath> ice_compress(bool __compress) const
    {
        return dynamic_cast<OptimalPath*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    }
    
    ::IceInternal::ProxyHandle<OptimalPath> ice_timeout(int __timeout) const
    {
        return dynamic_cast<OptimalPath*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    }
    
    ::IceInternal::ProxyHandle<OptimalPath> ice_connectionId(const ::std::string& __id) const
    {
        return dynamic_cast<OptimalPath*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    }
    
    ::IceInternal::ProxyHandle<OptimalPath> ice_encodingVersion(const ::Ice::EncodingVersion& __v) const
    {
        return dynamic_cast<OptimalPath*>(::IceProxy::Ice::Object::ice_encodingVersion(__v).get());
    }
    
    static const ::std::string& ice_staticId();

private: 
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

}

}

namespace RobotPath
{

class OptimalPath : virtual public ::Ice::Object
{
public:

    typedef OptimalPathPrx ProxyType;
    typedef OptimalPathPtr PointerType;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual ::RobotPath::edgeArray calcOptimalPath(const ::std::string&, const ::RobotPath::edgeArray&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___calcOptimalPath(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);

protected:
    virtual void __writeImpl(::IceInternal::BasicStream*) const;
    virtual void __readImpl(::IceInternal::BasicStream*);
    using ::Ice::Object::__writeImpl;
    using ::Ice::Object::__readImpl;
};

inline bool operator==(const OptimalPath& l, const OptimalPath& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

inline bool operator<(const OptimalPath& l, const OptimalPath& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

}

namespace RobotPath
{

template<class T>
class CallbackNC_OptimalPath_calcOptimalPath : public Callback_OptimalPath_calcOptimalPath_Base, public ::IceInternal::TwowayCallbackNC<T>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception&);
    typedef void (T::*Sent)(bool);
    typedef void (T::*Response)(const ::RobotPath::edgeArray&);

    CallbackNC_OptimalPath_calcOptimalPath(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallbackNC<T>(obj, cb != 0, excb, sentcb), _response(cb)
    {
    }

    virtual void completed(const ::Ice::AsyncResultPtr& __result) const
    {
        ::RobotPath::OptimalPathPrx __proxy = ::RobotPath::OptimalPathPrx::uncheckedCast(__result->getProxy());
        ::RobotPath::edgeArray __ret;
        try
        {
            __ret = __proxy->end_calcOptimalPath(__result);
        }
        catch(const ::Ice::Exception& ex)
        {
            ::IceInternal::CallbackNC<T>::exception(__result, ex);
            return;
        }
        if(_response)
        {
            (::IceInternal::CallbackNC<T>::_callback.get()->*_response)(__ret);
        }
    }

    private:

    Response _response;
};

template<class T> Callback_OptimalPath_calcOptimalPathPtr
newCallback_OptimalPath_calcOptimalPath(const IceUtil::Handle<T>& instance, void (T::*cb)(const ::RobotPath::edgeArray&), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_OptimalPath_calcOptimalPath<T>(instance, cb, excb, sentcb);
}

template<class T> Callback_OptimalPath_calcOptimalPathPtr
newCallback_OptimalPath_calcOptimalPath(T* instance, void (T::*cb)(const ::RobotPath::edgeArray&), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_OptimalPath_calcOptimalPath<T>(instance, cb, excb, sentcb);
}

template<class T, typename CT>
class Callback_OptimalPath_calcOptimalPath : public Callback_OptimalPath_calcOptimalPath_Base, public ::IceInternal::TwowayCallback<T, CT>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception& , const CT&);
    typedef void (T::*Sent)(bool , const CT&);
    typedef void (T::*Response)(const ::RobotPath::edgeArray&, const CT&);

    Callback_OptimalPath_calcOptimalPath(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallback<T, CT>(obj, cb != 0, excb, sentcb), _response(cb)
    {
    }

    virtual void completed(const ::Ice::AsyncResultPtr& __result) const
    {
        ::RobotPath::OptimalPathPrx __proxy = ::RobotPath::OptimalPathPrx::uncheckedCast(__result->getProxy());
        ::RobotPath::edgeArray __ret;
        try
        {
            __ret = __proxy->end_calcOptimalPath(__result);
        }
        catch(const ::Ice::Exception& ex)
        {
            ::IceInternal::Callback<T, CT>::exception(__result, ex);
            return;
        }
        if(_response)
        {
            (::IceInternal::Callback<T, CT>::_callback.get()->*_response)(__ret, CT::dynamicCast(__result->getCookie()));
        }
    }

    private:

    Response _response;
};

template<class T, typename CT> Callback_OptimalPath_calcOptimalPathPtr
newCallback_OptimalPath_calcOptimalPath(const IceUtil::Handle<T>& instance, void (T::*cb)(const ::RobotPath::edgeArray&, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_OptimalPath_calcOptimalPath<T, CT>(instance, cb, excb, sentcb);
}

template<class T, typename CT> Callback_OptimalPath_calcOptimalPathPtr
newCallback_OptimalPath_calcOptimalPath(T* instance, void (T::*cb)(const ::RobotPath::edgeArray&, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_OptimalPath_calcOptimalPath<T, CT>(instance, cb, excb, sentcb);
}

}

#include <IceUtil/PopDisableWarnings.h>
#endif