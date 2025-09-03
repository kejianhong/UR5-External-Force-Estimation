#ifndef LOGGER_H
#define LOGGER_H

#include <filesystem>
#include <string>
#include <unordered_map>

// ANSI 转义序列定义颜色
#define COLOR_RESET "\033[0m"
#define COLOR_RED "\033[31m"
#define COLOR_GREEN "\033[32m"
#define COLOR_YELLOW "\033[33m"
#define COLOR_BLUE "\033[34m"
#define COLOR_MAGENTA "\033[35m"
#define COLOR_CYAN "\033[36m"
#define COLOR_WHITE "\033[37m"

// 日志级别枚举
enum LogLevel {
  DEBUG,
  INFO,
  WARNING,
  ERROR,
  CRITICAL,
};

// 日志级别与颜色映射
const std::unordered_map<LogLevel, std::string> LOG_COLORS = {
    {DEBUG, COLOR_CYAN},
    {INFO, COLOR_GREEN},
    {WARNING, COLOR_YELLOW},
    {ERROR, COLOR_RED},
    {CRITICAL, COLOR_MAGENTA},
};

// 字符串映射
const std::unordered_map<int, LogLevel> INT_LOGLEVEL = {
    {1, DEBUG},
    {2, INFO},
    {3, WARNING},
    {4, ERROR},
    {5, CRITICAL},
};

// 全局日志级别
static LogLevel GLOBAL_LOG_LEVEL = DEBUG;

// 设置日志级别
void SetLogLevel(const LogLevel& level);

// 获取当前时间戳
std::string GetCurrentTimestamp();

// 日志函数
void LogMessage(const LogLevel& level, const std::string& message, const std::string& file, const int& line, const std::string& function);

// 宏定义简化日志调用
#define LOG(level, message) LogMessage(level, message, __FILE__, __LINE__, __func__)

#endif
