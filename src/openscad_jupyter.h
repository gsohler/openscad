
#ifndef XEUS_CALC_INTERPRETER_HPP
#define XEUS_CALC_INTERPRETER_HPP

#include "xeus/xinterpreter.hpp"
#include "nlohmann/json.hpp"
#include "openscad_jupyter_config.h"
#include "xeus/xrequest_context.hpp"

namespace nl = nlohmann;

namespace openscad_jupyter
{
    class XEUS_CALC_API interpreter : public xeus::xinterpreter
    {
    public:

        interpreter() = default;

        virtual ~interpreter() = default;

    private:

        void configure_impl() override;

        void execute_request_impl(send_reply_callback cb,
                                  int execution_counter,
                                  const std::string& code,
                                  xeus::execute_request_config config, 
                                  nl::json user_expressions) override;

        nl::json complete_request_impl(const std::string& code,
                                       int cursor_pos) override;

        nl::json inspect_request_impl(const std::string& code,
                                      int cursor_pos,
                                      int detail_level) override;

        nl::json is_complete_request_impl(const std::string& code) override;

        nl::json kernel_info_request_impl() override;

        void shutdown_request_impl() override;
    };

    using publish_type = std::function<void(const std::string& name, const std::string& text)>;

    XEUS_CALC_API std::string formating_expr(const std::string& expr);

    XEUS_CALC_API std::string parse_rpn(const std::string& infix, publish_type publish = [](const std::string& /*name*/, const std::string& /*text*/){});

    XEUS_CALC_API double compute_rpn(const std::string &expr, publish_type publish = [](const std::string& /*name*/, const std::string& /*text*/){});
}
#endif
