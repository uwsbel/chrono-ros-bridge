#ifndef CH_JSON_HELPER_H
#define CH_JSON_HELPER_H

#include <mutex>
#include <string>
#include <array>
#include <vector>

#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

namespace chrono {
namespace utils {

/**
 * This is a helper class to generate serialized JSON messages that can be passed to/from Chrono. The expectation the
 * replciate class for this object should be used on the Chrono side
 *
 * This works as follows:
 * - The ChJSONWriter implements << operators that correspond to the types provided by rapidjson
 * - The ChJSONWriter is responsible for interacting with rapidjson, generating JSON buffers that are suitable to be
 * read on the other side
 */
class ChJSONWriter {
  public:
    ChJSONWriter();

    //  Operators

    ChJSONWriter& operator<<(bool v);
    ChJSONWriter& operator<<(const int v);
    ChJSONWriter& operator<<(const long int v);
    ChJSONWriter& operator<<(const double v);
    ChJSONWriter& operator<<(const float v);
    ChJSONWriter& operator<<(unsigned int v);
    ChJSONWriter& operator<<(const char* v);
    ChJSONWriter& operator<<(std::string& v);
    ChJSONWriter& operator<<(const std::string& v);
    ChJSONWriter& operator<<(unsigned long v);
    ChJSONWriter& operator<<(unsigned long long v);
    ChJSONWriter& operator<<(ChJSONWriter&) { return *this; };

    ChJSONWriter& Key(const std::string& v);

    ChJSONWriter& StartObject(const std::string& type);
    ChJSONWriter& EndObject();

    std::string Finish();

  private:
    std::mutex m_mutex;
    rapidjson::StringBuffer m_buffer;
    rapidjson::Writer<rapidjson::StringBuffer> m_writer;
};

// ------------------------------------------------------------------------------------

class ChJSONReader {
  public:
    ChJSONReader();

    void Parse(const std::string& message);

    //  Operators

    ChJSONReader& operator>>(bool& v);
    ChJSONReader& operator>>(int& v);
    ChJSONReader& operator>>(long int& v);
    ChJSONReader& operator>>(double& v);
    ChJSONReader& operator>>(float& v);
    ChJSONReader& operator>>(unsigned int& v);
    ChJSONReader& operator>>(std::vector<uint8_t>& v);
    ChJSONReader& operator>>(std::string& v);
    ChJSONReader& operator>>(unsigned long& v);
    ChJSONReader& operator>>(unsigned long long& v);
    ChJSONReader& operator>>(std::array<double, 3>& v);
    ChJSONReader& operator>>(std::array<double, 4>& v);
    ChJSONReader& operator>>(ChJSONReader&) { return *this; };

    ChJSONReader& Next();
    ChJSONReader& Back();

    ChJSONReader& StartObject();
    ChJSONReader& GetObject();
    ChJSONReader& EndObject();

    bool HasMembers();

  private:
    rapidjson::Document m_d;
    rapidjson::Value::ConstValueIterator m_arr_iterator;
    rapidjson::Value::ConstMemberIterator m_obj_iterator;
};

}  // namespace utils
}  // namespace chrono

#endif
