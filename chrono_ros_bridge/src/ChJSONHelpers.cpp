#include "chrono_ros_bridge/ChJSONHelpers.h"

#define OUTPUT(type, v)             \
    std::scoped_lock lock(m_mutex); \
    m_writer.type(v);

#define _INPUT(TYPE, v, it, GET)           \
    v = m_obj_iterator->value.GET##TYPE(); \
    it++;
#define INPUT(type, v) _INPUT(type, v, m_obj_iterator, Get)

namespace chrono {
namespace utils {

ChJSONWriter::ChJSONWriter() : m_writer(m_buffer) {
    m_writer.StartArray();
}

ChJSONWriter& ChJSONWriter::operator<<(bool v) {
    OUTPUT(Bool, v);
    return *this;
}

ChJSONWriter& ChJSONWriter::operator<<(const int v) {
    OUTPUT(Int, v);
    return *this;
}

ChJSONWriter& ChJSONWriter::operator<<(const long int v) {
    OUTPUT(Int64, v);
    return *this;
}

ChJSONWriter& ChJSONWriter::operator<<(const double v) {
    OUTPUT(Double, v);
    return *this;
}

ChJSONWriter& ChJSONWriter::operator<<(const float v) {
    OUTPUT(Double, v);
    return *this;
}

ChJSONWriter& ChJSONWriter::operator<<(unsigned int v) {
    OUTPUT(Uint, v);
    return *this;
}

ChJSONWriter& ChJSONWriter::operator<<(const char* v) {
    OUTPUT(String, v);
    return *this;
}

ChJSONWriter& ChJSONWriter::operator<<(std::string& v) {
    OUTPUT(String, v.c_str());
    return *this;
}

ChJSONWriter& ChJSONWriter::operator<<(const std::string& v) {
    OUTPUT(String, v.c_str());
    return *this;
}

ChJSONWriter& ChJSONWriter::operator<<(unsigned long v) {
    OUTPUT(Uint64, v);
    return *this;
}

ChJSONWriter& ChJSONWriter::operator<<(unsigned long long v) {
    OUTPUT(Uint64, v);
    return *this;
}

ChJSONWriter& ChJSONWriter::Key(const std::string& v) {
    OUTPUT(Key, v.c_str());
    return *this;
}

ChJSONWriter& ChJSONWriter::StartObject(const std::string& type) {
    std::scoped_lock lock(m_mutex);
    m_writer.StartObject();

    m_writer.Key("type");
    m_writer.String(type.c_str());

    m_writer.Key("data");
    m_writer.StartObject();

    return *this;
}

ChJSONWriter& ChJSONWriter::EndObject() {
    std::scoped_lock lock(m_mutex);
    m_writer.EndObject();

    return *this;
}

std::string ChJSONWriter::Finish() {
    std::scoped_lock lock(m_mutex);

    m_writer.EndArray();
    std::string message(m_buffer.GetString());

    // Restart the buffer
    m_buffer.Clear();
    m_writer.Reset(m_buffer);
    m_writer.StartArray();

    return message;
}

// ------------------------------------------------------------------------------------

ChJSONReader::ChJSONReader() {}

void ChJSONReader::Parse(const std::string& message) {
    m_d.Parse(message.c_str());

    m_arr_iterator = m_d.Begin();
}

ChJSONReader& ChJSONReader::operator>>(bool& v) {
    INPUT(Bool, v);
    return *this;
}

ChJSONReader& ChJSONReader::operator>>(int& v) {
    INPUT(Int, v);
    return *this;
}

ChJSONReader& ChJSONReader::operator>>(long int& v) {
    INPUT(Int64, v);
    return *this;
}

ChJSONReader& ChJSONReader::operator>>(double& v) {
    INPUT(Double, v);
    return *this;
}

ChJSONReader& ChJSONReader::operator>>(float& v) {
    INPUT(Double, v);
    return *this;
}

ChJSONReader& ChJSONReader::operator>>(unsigned int& v) {
    INPUT(Uint, v);
    return *this;
}

ChJSONReader& ChJSONReader::operator>>(std::string& v) {
    INPUT(String, v);
    return *this;
}

ChJSONReader& ChJSONReader::operator>>(unsigned long& v) {
    INPUT(Uint64, v);
    return *this;
}

ChJSONReader& ChJSONReader::operator>>(unsigned long long& v) {
    INPUT(Uint64, v);
    return *this;
}

ChJSONReader& ChJSONReader::operator>>(std::array<double, 3>& v) {
    auto temp_arr_iterator = m_obj_iterator->value.GetArray().Begin();
    for (int i = 0; i < 3; i++, temp_arr_iterator++)
        v[i] = temp_arr_iterator->GetDouble();
    return *this;
}

ChJSONReader& ChJSONReader::operator>>(std::array<double, 4>& v) {
    auto temp_arr_iterator = m_obj_iterator->value.GetArray().Begin();
    for (int i = 0; i < 4; i++, temp_arr_iterator++)
        v[i] = temp_arr_iterator->GetDouble();
    return *this;
}

ChJSONReader& ChJSONReader::Next() {
    m_obj_iterator++;
    return *this;
}

ChJSONReader& ChJSONReader::Back() {
    m_obj_iterator--;
    return *this;
}

ChJSONReader& ChJSONReader::GetObject() {
    m_obj_iterator = m_obj_iterator->value.GetObject().MemberBegin();
    return *this;
}

ChJSONReader& ChJSONReader::StartObject() {
    m_obj_iterator = m_arr_iterator->MemberBegin();
    return *this;
}

ChJSONReader& ChJSONReader::EndObject() {
    m_arr_iterator++;
    return *this;
}

bool ChJSONReader::HasMembers() {
    return m_arr_iterator != m_d.End();
}

}  // namespace utils
}  // namespace chrono
