#include "..\include\gmath.hpp"

using namespace gmath;

quat::quat(const float a, const float b, const float c, const float d):
	data(new float[4]{ a, b, c, d }) {}

quat::quat(const __m128& data) :
	data(new float[4]) {
	_mm_store_ps(this->data, data);
}

quat::quat(const vec4& v):
	data(new float[4]) {
	data[0] = 0.0f;
	data[1] = v.data[0];
	data[2] = v.data[1];
	data[3] = v.data[2];
}

quat::quat(const quat& other):
	data(new float[4]) {
	data[0] = other.data[0];
	data[1] = other.data[1];
	data[2] = other.data[2];
	data[3] = other.data[3];
}

quat::quat(quat&& other):
	data(nullptr) {
	this->data = other.data;
	other.data = nullptr;
}

quat::~quat() {
	if (data != nullptr) {
		delete[] data;
		data = nullptr;
	}
}

float& quat::operator[](const uint32_t& i) const {
	return data[i];
}

quat& quat::operator=(const quat& other) {
	if (this != &other) {
		data[0] = other.data[0];
		data[1] = other.data[1];
		data[2] = other.data[2];
		data[3] = other.data[3];
	}

	return *this;
}

quat& quat::operator=(quat&& other) {
	if (this != &other) {
		delete[] data;

		data = other.data;
		other.data = nullptr;
	}

	return *this;
}

quat quat::operator-() const {
	const __m128 v = _mm_load_ps(this->data);
	return quat(_mm_xor_ps(v, _mm_set1_ps(-0.0)));
}

quat quat::conjugate() const {
	return quat( data[0], -data[1], -data[2], -data[3] );
}

float quat::norm() const {
	return sqrt(
		data[0] * data[0] + data[1] * data[1] +
		data[2] * data[2] + data[3] * data[3]
	);
}

quat quat::inverse() const {
	const float v2 = data[0] * data[0] + data[1] * data[1] +
		data[2] * data[2] + data[3] * data[3];
	return this->conjugate() / v2;
}

quat quat::normalize() const {
	const float v = this->norm();
	return *this / v;
}

vec4 quat::transform(const vec4& v) const {
	const quat q = (*this) * (quat(v) * (*this).inverse());
	return vec4(q.data[1], q.data[2], q.data[3], v.data[3]);
}

vec4 quat::transform(const vec4& v, const vec4& t) const {
	return v[3] == 0.0f ? this->transform(v) : t + this->transform(v);
}

std::string quat::toString() const {
	return std::string("a: ") + std::to_string(data[0]) +
		std::string(" b: ") + std::to_string(data[1]) +
		std::string(" c: ") + std::to_string(data[2]) +
		std::string(" d: ") + std::to_string(data[3]);
}

quat gmath::operator*(const quat& q1, const quat& q2) {
	const __m128 q1Data = _mm_load_ps(q1.data);
	const __m128 q2Data = _mm_load_ps(q2.data);

	const __m128 aaaa = _mm_replicate_x_ps(q1Data);
	const __m128 bbbb = _mm_replicate_y_ps(q1Data);
	const __m128 cccc = _mm_replicate_z_ps(q1Data);
	const __m128 dddd = _mm_replicate_w_ps(q1Data);

	// contains a1a2, a1b2, a1c2, a1d2
	float prod1[4];
	_mm_store_ps(&prod1[0], _mm_mul_ps(aaaa, q2Data));

	// contains b1a2, b1b2, b1c2, b1d2
	float prod2[4];
	_mm_store_ps(&prod2[0], _mm_mul_ps(bbbb, q2Data));

	// contains c1a2, c1b2, c1c2, c1d2
	float prod3[4];
	_mm_store_ps(&prod3[0], _mm_mul_ps(cccc, q2Data));

	// contains d1a2, d1b2, d1c2, d1d2
	float prod4[4];
	_mm_store_ps(&prod4[0], _mm_mul_ps(dddd, q2Data));

	const __m128 val1 = _mm_set_ps(prod1[3], prod1[2], prod1[1], prod1[0]);
	const __m128 val2 = _mm_set_ps(prod2[2], -prod2[3], prod2[0], -prod2[1]);
	const __m128 val3 = _mm_set_ps(-prod3[1], prod3[0], prod3[3], -prod3[2]);
	const __m128 val4 = _mm_set_ps(prod4[0], prod4[1], -prod4[2], -prod4[3]);

	return quat(
		_mm_add_ps(_mm_add_ps(_mm_add_ps(val1, val2), val3), val4)
	);
}

quat gmath::operator*(const float& s, const quat& q) {
	const __m128 a = _mm_set_ps1(s);
	const __m128 b = _mm_load_ps(q.data);

	return quat(_mm_mul_ps(a, b));
}

quat gmath::operator*(const quat& q, const float& s) {
	const __m128 a = _mm_set_ps1(s);
	const __m128 b = _mm_load_ps(q.data);

	return quat(_mm_mul_ps(a, b));
}

quat gmath::operator/(const quat& q, const float& s) {
	return (1.0f / s) * q;
}

quat gmath::operator+(const quat& q1, const quat& q2) {
	const __m128 a = _mm_load_ps(q1.data);
	const __m128 b = _mm_load_ps(q2.data);

	return quat(_mm_add_ps(a, b));
}

quat gmath::operator-(const quat& q1, const quat& q2) {
	const __m128 a = _mm_load_ps(q1.data);
	const __m128 b = _mm_load_ps(q2.data);

	return quat(_mm_sub_ps(a, b));
}