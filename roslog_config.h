#ifndef ROSLOG_CONFIG_H
#define ROSLOG_CONFIG_H

#define LOG_LEVEL_NOOUTPUT 0
#define LOG_LEVEL_ERRORS 1
#define LOG_LEVEL_INFOS 2
#define LOG_LEVEL_DEBUG 3

#define LOG_INFO_LN(msg,   arguments...) Log.Print(msg, ## arguments)

#if (LOG_LEVEL >= LOG_LEVEL_DEBUG)
  #define LOG_DEBUG_LN(msg,   arguments...) Log.Print(msg, ## arguments)
#else
  #define LOG_DEBUG_LN(msg,   arguments...)
#endif

class Logging {
private:
  char out[40];
  
public:
  void Print(const char msg[], ...){
      va_list args;
      va_start(args, msg);
      snprintf(out,40,msg,args);
      nh.loginfo(out);
  }
};

Logging Log = Logging();
#endif /** ROSLOG_CONFIG_H **/
