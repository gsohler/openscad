/***************************************************************************
* Copyright (c) 2019, Sylvain Corlay, Johan Mabille, Wolf Vollprecht       *
* Copyright (c) 2019, QuantStack                                           *
*                                                                          *
* Distributed under the terms of the BSD 3-Clause License.                 *
*                                                                          *
* The full license is in the file LICENSE, distributed with this software. *
****************************************************************************/

#include <iostream>
#include <vector>
#include <sstream>
#include <stack>
#include <cctype>

#include "xeus/xinterpreter.hpp"

#include "openscad_jupyter.h"

namespace openscad_jupyter
{
    void interpreter::configure_impl()
    {
    }

    void interpreter::execute_request_impl(send_reply_callback cb,
                                           int execution_counter,
                                           const std::string& code,
                                           xeus::execute_request_config config,
                                           nl::json user_expression)
    {
        nl::json jresult;
        try
        {
            nl::json pub_data;
            publish_stream("stdout", "normal output\n");
            pub_data["text/plain"] = "my result"; // result;
            publish_execution_result(execution_counter, std::move(pub_data), nl::json::object());
            jresult["status"] = "ok";
            jresult["payload"] = nl::json::array();
            jresult["user_expressions"] = nl::json::object();
            cb(jresult);
        }
        catch (const std::runtime_error& err)
        {
            publish_stream("stderr", err.what());
            jresult["status"] = "error";
            cb(jresult);
        }
    }

    nl::json interpreter::complete_request_impl(const std::string& /*code*/, int /*cursor_pos*/)
    {
        nl::json jresult;
        jresult["status"] = "ok";
        return jresult;
    };

    nl::json interpreter::inspect_request_impl(const std::string& /*code*/,
                                               int /*cursor_pos*/,
                                               int /*detail_level*/)
    {
        nl::json jresult;
        jresult["status"] = "ok";
        return jresult;
    };

    nl::json interpreter::is_complete_request_impl(const std::string& /*code*/)
    {
        nl::json jresult;
        jresult["status"] = "complete";
        return jresult;
    };

    nl::json interpreter::kernel_info_request_impl()
    {
        nl::json result;
        result["implementation"] = "openscad";
        result["implementation_version"] = "0.1.0";
        std::string banner = "PythonSCAD\n"
        " goes Jupyter";
        result["banner"] = banner;
        result["language_info"]["name"] = "openscad";
        result["language_info"]["version"] = "";
        result["language_info"]["mimetype"] = "";
        result["language_info"]["file_extension"] = "py";
        return result;
    }

    void interpreter::shutdown_request_impl()
    {
    }
}
