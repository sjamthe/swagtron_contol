#ifndef LOGGER_CONFIG_H
#define LOGGER_CONFIG_H

#define LOG_LEVEL_NOOUTPUT 0
#define LOG_LEVEL_ERRORS 1
#define LOG_LEVEL_INFOS 2
#define LOG_LEVEL_DEBUG 3

#define LOG_INIT_STREAM(level, stream) Log.Init(level, stream)
#define LOG_INFO_LN(msg,   arguments...) Log.Print(msg, ## arguments)

#if (LOG_LEVEL >= LOG_LEVEL_DEBUG)
  #define LOG_DEBUG_LN(msg,   arguments...) Log.Print(msg, ## arguments)
#else
  #define LOG_DEBUG_LN(msg,   arguments...)
#endif

static const char* BL = "\n";
class Logging {
private:
  Stream*  _p_output_stream;
    uint8_t _u8_logLevel;
public:
    /*! 
   * default Constructor
   */
    Logging(): _p_output_stream(&Serial), _u8_logLevel(LOG_LEVEL_DEBUG){} ;
  
    void Init(int level, Stream*  arg_p_output_stream)
    {
      _p_output_stream = arg_p_output_stream;
      _p_output_stream.begin(LOG_SPEED);
      _u8_logLevel = constrain(level,LOG_LEVEL_NOOUTPUT,LOG_LEVEL_DEBUG);
    }
    
    void Print(const char msg[], ...){
        va_list args;
        va_start(args, msg);
        print(msg,args);
        _p_output_stream->print(BL);
        _p_output_stream->flush();
    }

private:
    void printArg(char arg_s8Char, va_list& args) {
      if (arg_s8Char == '%') {
        _p_output_stream->print(arg_s8Char);
      }
      if( arg_s8Char == 's' ) {
        register char *s = (char *)va_arg( args, int );
        _p_output_stream->print(s);
      }
      if( arg_s8Char == 'd' || arg_s8Char == 'i') {
        _p_output_stream->print(va_arg( args, int ),DEC);
      }
      if( arg_s8Char == 'u') {
        _p_output_stream->print((unsigned int) va_arg( args, int ),DEC);
      }
      if( arg_s8Char == 'x' ) {
        _p_output_stream->print("0x");
        _p_output_stream->print(va_arg( args, int ),HEX);
      }
      if( arg_s8Char == 'X' ) {
        _p_output_stream->print("0x");
        _p_output_stream->print(va_arg( args, int ),HEX);
      }
      if( arg_s8Char == 'b' ) {
        _p_output_stream->print(va_arg( args, int ),BIN);
      }
      if( arg_s8Char == 'B' ) {
        _p_output_stream->print("0b");
        _p_output_stream->print(va_arg( args, int ),BIN);
      }
      if( arg_s8Char == 'l' ) {
        _p_output_stream->print(va_arg( args, long ),DEC);
      }
    
      if( arg_s8Char == 'c' ) {
        char s = (char)va_arg( args, int );
        _p_output_stream->print(s);
      }
      if( arg_s8Char == 't' ) {
        if (va_arg( args, int ) == 1) {
          _p_output_stream->print("T");
        }
        else {
          _p_output_stream->print("F");
        }
      }
      if( arg_s8Char == 'T' ) {
        if (va_arg( args, int ) == 1) {
          _p_output_stream->print("true");
        }
        else {
          _p_output_stream->print("false");
        }
      }
      if( arg_s8Char == 'f' ) {
        _p_output_stream->print((float) va_arg( args, double ), 8);
      }
    }

    void print(const char *format, va_list args) {
      //
      // loop through format string
      for (; *format != 0; ++format) {
        if (*format == '%') {
          ++format;
          if (*format == '\0') break;
          printArg(*format, args);
        }
        else
        {
          _p_output_stream->print(*format);
        }
      }
      _p_output_stream->flush();
    }
};

extern Logging Log;  

Logging Log = Logging();
#endif /** LOGGER_CONFIG_H **/
