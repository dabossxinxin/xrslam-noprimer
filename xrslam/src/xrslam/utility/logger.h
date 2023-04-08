#include <string>
#include <vector>
#include <filesystem>
#include <glog/logging.h>

namespace xrslam {

	inline void TurnOnGlog(const char* pszAppName, const std::string& strLogDir) {
		const char* _pszAppName = pszAppName;
		std::string _pstrLogDir = strLogDir;
		_pstrLogDir += "/";
		
		std::time_t const now_c = std::time(0);
		std::stringstream out;
		out << std::put_time(std::localtime(&now_c), "%F");
		std::string strDay = out.str();

		_pstrLogDir += strDay;
		
		std::error_code ec;
		std::filesystem::create_directories(_pstrLogDir, ec);

		google::InitGoogleLogging(pszAppName);
		FLAGS_logbufsecs = 3;
		FLAGS_max_log_size = 10;
		FLAGS_stop_logging_if_full_disk = true;
		FLAGS_logtostderr = true;

		google::SetLogFilenameExtension(".log");
		google::SetLogDestination(google::GLOG_INFO, (_pstrLogDir + "\\Info" + "-").c_str());
		google::SetLogDestination(google::GLOG_WARNING, (_pstrLogDir + "\\Warning" + "-").c_str());
		google::SetLogDestination(google::GLOG_ERROR, (_pstrLogDir + "\\Error" + "-").c_str());
	}

	inline void TurnOffGlog() {
		google::ShutdownGoogleLogging();
	}

	class Logger {
	public:
		static Logger& instance() {
			static Logger logger;
			return logger;
		}
		Logger(const Logger&) = delete;
		Logger &operator=(const Logger&) = delete;

		void pushMessage(std::string msg) {
			this->message_list.emplace_back(msg);
		}

		std::vector<std::string> getAllMessage() {
			std::vector<std::string> ret(this->message_list);
			this->message_list.clear();
			return ret;
		}

	private:
		Logger() {
			this->message_list = std::vector<std::string>();
		}

		~Logger() = default;

		std::vector<std::string> message_list;
	};
} // namespace xrslam
