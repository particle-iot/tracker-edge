/*
 * Copyright (c) 2020 Particle Industries, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <math.h>

#include <memory>

typedef enum config_node_type_t {
    CONFIG_NODE_TYPE_UNKNOWN,
    CONFIG_NODE_TYPE_NULL,
    CONFIG_NODE_TYPE_BOOL,
    CONFIG_NODE_TYPE_INT,
    CONFIG_NODE_TYPE_FLOAT,
    CONFIG_NODE_TYPE_STRING,
    CONFIG_NODE_TYPE_STRING_ENUM,
    CONFIG_NODE_TYPE_ARRAY,
    CONFIG_NODE_TYPE_OBJECT
} config_node_type_t;

// some default set/get callbacks that work directly wth memory locatons and no extra control
int config_get_int32_cb(int32_t &value, const void *context);
int config_set_int32_cb(int32_t value, const void *context);

int config_get_bool_cb(bool &value, const void *context);
int config_set_bool_cb(bool value, const void *context);

int config_get_float_cb(double &value, const void *context);
int config_set_float_cb(double value, const void *context);

int config_get_string_cb(const char * &value, const void *context);
int config_set_string_cb(const char *value, const void *context);

class ConfigNode
{
    public:
        ConfigNode(const char *name=nullptr, config_node_type_t node_type=CONFIG_NODE_TYPE_UNKNOWN) : _name(name), _type(node_type) {}
        virtual ~ConfigNode() {}
        config_node_type_t type() {return _type;}
        const char * name() {return _name;}
    private:
        const char *_name;
        config_node_type_t _type;
};

// allocates a copy of of ConfigNode derived class via std::shared_ptr
// intended for use with std::initializer_list or similar to allocate a new
// copy during initialization
class ConfigNodeAllocator
{
    public:
        template <class T>
        ConfigNodeAllocator(T node) : node(std::make_shared<T>(node)) {}
        
        std::shared_ptr<ConfigNode> get() const {return node;};
    private:
        std::shared_ptr<ConfigNode> node;
};

template <class T, config_node_type_t NODE_T>
class ConfigLeaf : public ConfigNode
{
    public:
        ConfigLeaf(const char *name,
            std::function<int(T &value, const void *context)> get_cb=nullptr,
            std::function<int(T value, const void *context)> set_cb=nullptr,
            void *context=nullptr,
            void *wcontext=nullptr) :
            ConfigNode(name, NODE_T),
            get_cb(get_cb),
            set_cb(set_cb),
            context(context),
            wcontext(wcontext)
        {
        }

        virtual bool check(T value);
        int get(T &value);
        int set(T value);
    protected:
        std::function<int(T &value, const void *context)> get_cb;
        std::function<int(T value, const void *context)> set_cb;
        const void *context;
        const void *wcontext;
};

class ConfigBool : public ConfigLeaf<bool, CONFIG_NODE_TYPE_BOOL>
{
    public:
        ConfigBool(const char *name,
            std::function<int(bool &value, const void *context)> get_cb=nullptr,
            std::function<int(bool value, const void *context)> set_cb=nullptr,
            void *context=nullptr,
            void *wcontext=nullptr) :
            ConfigLeaf<bool, CONFIG_NODE_TYPE_BOOL>(name, get_cb, set_cb, context, wcontext)
        {
        }

        ConfigBool(const char *name,
            void *context) :
            ConfigLeaf<bool, CONFIG_NODE_TYPE_BOOL>(name, config_get_bool_cb, config_set_bool_cb, context)
        {
        }
};

class ConfigInt : public ConfigLeaf<int32_t, CONFIG_NODE_TYPE_INT>
{
    public:
        ConfigInt(const char *name,
            std::function<int(int32_t &value, const void *context)> get_cb=nullptr,
            std::function<int(int32_t value, const void *context)> set_cb=nullptr,
            void *context=nullptr,
            void *wcontext=nullptr,
            int32_t range_min=INT32_MIN,
            int32_t range_max=INT32_MAX) :
            ConfigLeaf<int32_t, CONFIG_NODE_TYPE_INT>(name, get_cb, set_cb, context, wcontext),
            range_min(range_min),
            range_max(range_max)
        {
        }

        ConfigInt(const char *name,
            void *context,
            int32_t range_min=INT32_MIN,
            int32_t range_max=INT32_MAX) :
            ConfigLeaf<int32_t, CONFIG_NODE_TYPE_INT>(name, config_get_int32_cb, config_set_int32_cb, context),
            range_min(range_min),
            range_max(range_max)
        {
        }

        bool check(int32_t value) { return (value >= range_min && value <= range_max); };

        ConfigInt &max(int32_t value) {range_max = value; return *this;}
        ConfigInt &min(int32_t value) {range_min = value; return *this;}
    private:
        int32_t range_min;
        int32_t range_max;
};

class ConfigFloat : public ConfigLeaf<double, CONFIG_NODE_TYPE_FLOAT>
{
    public:
        ConfigFloat(const char *name,
            std::function<int(double &value, const void *context)> get_cb=nullptr,
            std::function<int(double value, const void *context)> set_cb=nullptr,
            void *context=nullptr,
            void *wcontext=nullptr,
            double range_min=NAN,
            double range_max=NAN) :
            ConfigLeaf<double, CONFIG_NODE_TYPE_FLOAT>(name, get_cb, set_cb, context, wcontext),
            range_min(range_min),
            range_max(range_max)
        {
        }

        ConfigFloat(const char *name,
            void *context,
            double range_min=NAN,
            double range_max=NAN) :
            ConfigLeaf<double, CONFIG_NODE_TYPE_FLOAT>(name, config_get_float_cb, config_set_float_cb, context),
            range_min(range_min),
            range_max(range_max)
        {
        }

        bool check(double value);

        ConfigFloat &max(double value) {range_max = value; return *this;}
        ConfigFloat &min(double value) {range_min = value; return *this;}
    private:
        double range_min;
        double range_max;
};

class ConfigString : public ConfigLeaf<const char *, CONFIG_NODE_TYPE_STRING>
{
    public:
        ConfigString(const char *name,
            std::function<int(const char * &value, const void *context)> get_cb=nullptr,
            std::function<int(const char *value, const void *context)> set_cb=nullptr,
            void *context=nullptr,
            void *wcontext=nullptr,
            size_t size=0) :
            ConfigLeaf<const char *, CONFIG_NODE_TYPE_STRING>(name, get_cb, set_cb, context, wcontext),
            _size(size)
        {
        }

        ConfigString(const char *name,
            void *context,
            size_t size=0) :
            ConfigLeaf<const char *, CONFIG_NODE_TYPE_STRING>(name, config_get_string_cb, config_set_string_cb, context),
            _size(size)
        {
        }
        
        bool check(const char *value);

        ConfigString &size(size_t value) {_size = value; return *this;}
    private:
        size_t _size;
};

class ConfigStringEnum : public ConfigLeaf<int32_t, CONFIG_NODE_TYPE_STRING_ENUM>
{
    public:
        ConfigStringEnum(const char *name,
            Vector<std::pair<const char *, int32_t>> enums,
            std::function<int(int32_t &value, const void *context)> get_cb=nullptr,
            std::function<int(int32_t value, const void *context)> set_cb=nullptr,
            void *context=nullptr,
            void *wcontext=nullptr) :
            ConfigLeaf<int32_t, CONFIG_NODE_TYPE_STRING_ENUM>(name, get_cb, set_cb, context, wcontext),
            enums(enums)
        {
        }

        ConfigStringEnum(const char *name,
            Vector<std::pair<const char *, int32_t>> enums,
            void *context) :
            ConfigLeaf<int32_t, CONFIG_NODE_TYPE_STRING_ENUM>(name, config_get_int32_cb, config_set_int32_cb, context),
            enums(enums)
        {
        }

        int get(const char * &value);
        int set(const char * value);
    private:
        Vector<std::pair<const char *, int32_t>> enums;
};

class ConfigObject : public ConfigNode
{
    public:
        ConfigObject(const char *name,
            std::initializer_list<ConfigNodeAllocator> children,
            std::function<int(bool write, const void *context)> enter_cb=nullptr,
            std::function<int(bool write, int status, const void *context)> exit_cb=nullptr,
            void *context=nullptr,
            void *wcontext=nullptr) :
        ConfigNode(name, CONFIG_NODE_TYPE_OBJECT),
        enter_cb(enter_cb),
        exit_cb(exit_cb),
        context(context),
        wcontext(wcontext)
        {
            _children.reserve(children.size()); // minimize memory allocation
            for(auto child : children)
            {
                _children.append(child.get());
            }
        }

        int child_count() { return _children.size(); }
        ConfigNode *child(const char *name);
        ConfigNode *child(int position) { return (position < child_count()) ? _children[position].get() : nullptr; }
        int enter(bool write);
        int exit(bool write, int status);
    private:
        Vector<std::shared_ptr<ConfigNode>> _children;
        std::function<int(bool write, const void *context)> enter_cb;
        std::function<int(bool write, int status, const void *context)> exit_cb;
        const void *context;
        const void *wcontext;
};

template <class T, config_node_type_t NODE_T>
bool ConfigLeaf<T, NODE_T>::check(T value)
{
    return true;
}

template <class T, config_node_type_t NODE_T>
int ConfigLeaf<T, NODE_T>::set(T value)
{
    if(!set_cb)
    {
        return -EPERM;
    }

    if(!check(value))
    {
        return -EDOM;
    }

    return set_cb(value, wcontext ? wcontext : context);
}

template <class T, config_node_type_t NODE_T>
int ConfigLeaf<T, NODE_T>::get(T &value)
{
    if(!get_cb)
    {
        return -EPERM;
    }

    return get_cb(value, context);
}
