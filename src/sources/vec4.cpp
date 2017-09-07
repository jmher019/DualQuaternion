#include "..\include\gmath.hpp"

using namespace gmath;

vec4::vec4(const float& x, const float& y, const float& z, const float& w):
	data(new float[4]{ x, y, z, w}) {
}

vec4::vec4(const __m128& data):
	data(new float[4]) {
	_mm_store_ps(this->data, data);
}

vec4::vec4(const vec4& other):
	data(new float[4]) {
	data[0] = other.data[0];
	data[1] = other.data[1];
	data[2] = other.data[2];
	data[3] = other.data[3];
}

vec4::vec4(vec4&& other):
	data(nullptr) {
	data = other.data;
	other.data = nullptr;
}

vec4::~vec4() {
	if (data != nullptr) {
		delete[] data;
		data = nullptr;
	}
}

vec4& vec4::operator=(const vec4& other) {
	if (this != &other) {
		data[0] = other.data[0];
		data[1] = other.data[1];
		data[2] = other.data[2];
		data[3] = other.data[3];
	}

	return *this;
}

vec4& vec4::operator=(vec4&& other) {
	if (this != &other) {
		delete[] data;
		data = other.data;
		other.data = nullptr;
	}

	return *this;
}

float& vec4::operator[](const uint32_t& i) const {
	return data[i];
}

vec4 vec4::operator-() const {
	const __m128 v = _mm_load_ps(this->data);
	return vec4(_mm_xor_ps(v, _mm_set1_ps(-0.0)));
}

float vec4::dot(const vec4& other) const {
	const vec4 result((*this) * other);
	return result[0] + result[1] + result[2] + result[3];
}

float vec4::dot(const vec4& v1, const vec4& v2) {
	const vec4 result(v1 * v2);
	return result[0] + result[1] + result[2] + result[3];
}

float vec4::magnitude() const {
	return sqrt(this->magnitude2());
}

float vec4::magnitude2() const {
	return this->dot(*this);
}

vec4 vec4::multiply(const vec4& other) const {
	return ((*this) * other);
}

vec4 vec4::normalize() const {
	const float v = this->magnitude();
	return *this / v;
}

std::string vec4::toString() const {
	return std::string("x: ") + std::to_string(data[0]) +
		std::string(" y: ") + std::to_string(data[1]) +
		std::string(" z: ") + std::to_string(data[2]) +
		std::string(" w: ") + std::to_string(data[3]);
}

vec4 vec4::multiply(const vec4& v1, const vec4& v2) {
	return v1 * v2;
}

vec4 gmath::operator*(const vec4& v1, const vec4& v2) {
	const __m128 a = _mm_load_ps(v1.data);
	const __m128 b = _mm_load_ps(v2.data);

	return vec4(_mm_mul_ps(a, b));
}

vec4 gmath::operator*(const float& s, const vec4& v) {
	const __m128 a = _mm_set_ps1(s);
	const __m128 b = _mm_load_ps(v.data);

	return vec4(_mm_mul_ps(a, b));
}

vec4 gmath::operator*(const vec4& v, const float& s) {
	const __m128 a = _mm_set_ps1(s);
	const __m128 b = _mm_load_ps(v.data);

	return vec4(_mm_mul_ps(a, b));
}

vec4 gmath::operator/(const vec4& v, const float& s) {
	return (1.0f / s) * v;
}

vec4 gmath::operator+(const vec4& v1, const vec4& v2) {
	const __m128 a = _mm_load_ps(v1.data);
	const __m128 b = _mm_load_ps(v2.data);

	return vec4(_mm_add_ps(a, b));
}

vec4 gmath::operator-(const vec4& v1, const vec4& v2) {
	const __m128 a = _mm_load_ps(v1.data);
	const __m128 b = _mm_load_ps(v2.data);

	return vec4(_mm_sub_ps(a, b));
}