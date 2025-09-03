#include "Log/logger.h"

#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>
#include <unordered_map>

// 获取当前时间戳
std::string GetCurrentTimestamp() {
  const auto now = std::chrono::system_clock::now();
  const auto now_time_t = std::chrono::system_clock::to_time_t(now);
  const auto now_tm = *std::localtime(&now_time_t);
  const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
  std::ostringstream oss;
  oss << std::put_time(&now_tm, "%Y-%m-%d %H:%M:%S") << "," << std::setfill('0') << std::setw(3) << now_ms.count();
  return oss.str();
}

// 设置日志级别
void SetLogLevel(const LogLevel& level) {
  GLOBAL_LOG_LEVEL = level;
}

// 日志函数
void LogMessage(const LogLevel& level, const std::string& message, const std::string& file, const int& line, const std::string& function) {
  if (level < GLOBAL_LOG_LEVEL) {
    return;
  }

  // 获取日志级别对应的颜色
  const std::string& color = LOG_COLORS.at(level);
  std::string levelStr;

  // 日志级别字符串
  switch (level) {
    case DEBUG:
      levelStr = "DEBUG";
      break;
    case INFO:
      levelStr = "INFO";
      break;
    case WARNING:
      levelStr = "WARNING";
      break;
    case ERROR:
      levelStr = "ERROR";
      break;
    case CRITICAL:
      levelStr = "CRITICAL";
      break;
    default:
      levelStr = "UNKNOWN";
      break;
  }

  // 提取文件名（去掉路径）
  const std::string fileName = std::filesystem::path(file).filename().string();
  // 格式化日志消息
  std::ostringstream logStream;
  logStream << GetCurrentTimestamp() << "  "
            << "[" << levelStr << "] "
            << "[" << fileName << "] "
            << "[" << function << ":" << line << "] " << message;

  // 添加颜色并输出日志
  std::cout << color << logStream.str() << COLOR_RESET << std::endl;
}
