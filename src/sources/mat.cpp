#include "..\include\gmath.hpp"

using namespace gmath;

mat::mat(const vec4& c1, const vec4& c2, const vec4& c3, const vec4& c4):
	data(new vec4[4]{ c1, c2, c3, c4 }) {
}

mat::mat(const mat& other):
	data(new vec4[4]) {
	data[0] = vec4(other[0]);
	data[1] = vec4(other[1]);
	data[2] = vec4(other[2]);
	data[3] = vec4(other[3]);
}

mat::mat(mat&& other):
	data(nullptr) {
	this->data = other.data;
	other.data = nullptr;
}

mat::~mat() {
	if (data != nullptr) {
		delete[] data;
		data = nullptr;
	}
}

vec4& mat::operator[](const uint32_t& i) const {
	return data[i];
}

mat& mat::operator=(const mat& other) {
	if (this != &other) {
		data[0] = vec4(other[0]);
		data[1] = vec4(other[1]);
		data[2] = vec4(other[2]);
		data[3] = vec4(other[3]);
	}

	return *this;
}

mat& mat::operator=(mat&& other) {
	if (this != &other) {
		delete[] data;
		this->data = other.data;
		other.data = nullptr;
	}

	return *this;
}

std::string mat::toString() const {
	return std::string("col 1: ") + data[0].toString() +
		std::string("\n col 2: ") + data[1].toString() +
		std::string("\n col 3: ") + data[2].toString() +
		std::string("\n col 4: ") + data[3].toString();
}

mat mat::translate(const vec4& t) {
	return mat(
		vec4(1.0f),
		vec4(0.0f, 1.0f),
		vec4(0.0f, 0.0f, 1.0f),
		vec4(t.data[0], t.data[1], t.data[2], 1.0f)
	);
}

mat mat::rotateX(const float& radians) {
	const float COS = cosf(radians);
	const float SIN = sinf(radians);

	return mat(
		vec4(1.0f),
		vec4(0.0f, COS, SIN),
		vec4(0.0F, -SIN, COS)
	);
}

mat mat::rotateY(const float& radians) {
	const float COS = cosf(radians);
	const float SIN = sinf(radians);

	return mat(
		vec4(COS, 0.0f, -SIN),
		vec4(0.0f, 1.0f),
		vec4(SIN, 0.0f, COS)
	);
}

mat mat::rotateZ(const float& radians) {
	const float COS = cosf(radians);
	const float SIN = sinf(radians);

	return mat(
		vec4(COS, SIN),
		vec4(-SIN, COS),
		vec4(0.0f, 0.0f, 1.0f)
	);
}

mat mat::rotate(const vec4& a, const float& radians) {
	// assume user passes a unit direction vector, a
	// TODO: replace with quaternion
	const float COS = cosf(radians / 2.0f);
	const float SIN = sinf(radians / 2.0f);
	const vec4 v1(COS, SIN, SIN, SIN);
	const vec4 v2(1.0f, a[0], a[1], a[2]);
	const vec4 q(v1 * v2);

	const __m128 colvec = _mm_load_ps(q.data);

	const __m128 iiii = _mm_replicate_y_ps(colvec);
	const __m128 jjjj = _mm_replicate_z_ps(colvec);
	const __m128 kkkk = _mm_replicate_w_ps(colvec);

	const __m128 col = _mm_load_ps(q.data);

	// technically only need to do 9 mul but doing 12
	// contains qiqr, qi^2, qiqj, qiqk
	const vec4 mul1(_mm_mul_ps(iiii, col));
	// contains qjqr, _ , qj^2, qjqk
	const vec4 mul2(_mm_mul_ps(jjjj, col));
	// contains qkqr, _ , _ , qk^2
	const vec4 mul3(_mm_mul_ps(kkkk, col));

	return mat(
		vec4(1.0f - 2.0f * (mul2[2] + mul3[3]), 2.0f * (mul1[2] + mul3[0]), 2.0f * (mul1[3] - mul2[0])),
		vec4(2.0f * (mul1[2] - mul3[0]), 1.0f - 2.0f * (mul1[1] + mul3[3]), 2.0f * (mul2[3] + mul1[0])),
		vec4(2.0f * (mul1[3] + mul2[0]), 2.0f * (mul2[3] - mul1[0]), 1.0f - 2.0f * (mul1[1] + mul2[2]))
	);
}

mat mat::transform(const vec4& a, const float& radians, const vec4& t) {
	const mat result(mat::rotate(a, radians));

	return mat(
		vec4(result[0]),
		vec4(result[1]),
		vec4(result[2]),
		vec4(t.data[0], t.data[1], t.data[2], 1.0f)
	);
}

vec4 gmath::operator*(const mat& m, const vec4& v) {
	const __m128 colvec = _mm_load_ps(v.data);

	const __m128 xxxx = _mm_replicate_x_ps(colvec);
	const __m128 yyyy = _mm_replicate_y_ps(colvec);
	const __m128 zzzz = _mm_replicate_z_ps(colvec);
	const __m128 wwww = _mm_replicate_w_ps(colvec);

	const __m128 col1 = _mm_load_ps(m[0].data);
	const __m128 col2 = _mm_load_ps(m[1].data);
	const __m128 col3 = _mm_load_ps(m[2].data);
	const __m128 col4 = _mm_load_ps(m[3].data);

	const __m128 xMCol1 = _mm_mul_ps(col1, xxxx);
	const __m128 xMCol2 = _mm_mul_ps(col2, yyyy);
	const __m128 xMCol3 = _mm_mul_ps(col3, zzzz);
	const __m128 xMCol4 = _mm_mul_ps(col4, wwww);

	return vec4(_mm_add_ps(_mm_add_ps(_mm_add_ps(xMCol1, xMCol2), xMCol3), xMCol4));
}

mat gmath::operator*(const mat& m1, const mat& m2) {
	return mat(m1 * m2[0], m1 * m2[1], m1 * m2[2], m1 * m2[3]);
}

mat gmath::operator*(const float& s, const mat& m) {
	const __m128 ssss = _mm_set_ps(s, s, s, s);

	const __m128 col1 = _mm_load_ps(m.data[0].data);
	const __m128 col2 = _mm_load_ps(m.data[1].data);
	const __m128 col3 = _mm_load_ps(m.data[2].data);
	const __m128 col4 = _mm_load_ps(m.data[3].data);

	return mat(
		vec4(_mm_mul_ps(ssss, col1)),
		vec4(_mm_mul_ps(ssss, col2)),
		vec4(_mm_mul_ps(ssss, col3)),
		vec4(_mm_mul_ps(ssss, col4))
	);
}

mat gmath::operator*(const mat& m, const float& s) {
	const __m128 ssss = _mm_set_ps(s, s, s, s);

	const __m128 col1 = _mm_load_ps(m.data[0].data);
	const __m128 col2 = _mm_load_ps(m.data[1].data);
	const __m128 col3 = _mm_load_ps(m.data[2].data);
	const __m128 col4 = _mm_load_ps(m.data[3].data);

	return mat(
		vec4(_mm_mul_ps(ssss, col1)),
		vec4(_mm_mul_ps(ssss, col2)),
		vec4(_mm_mul_ps(ssss, col3)),
		vec4(_mm_mul_ps(ssss, col4))
	);
}

mat gmath::operator/(const mat& m, const float& s) {
	return (1.0f / s) * m;
}

mat gmath::operator+(const mat& m1, const mat& m2) {
	const __m128 a1 = _mm_load_ps(m1.data[0].data);
	const __m128 a2 = _mm_load_ps(m1.data[1].data);
	const __m128 a3 = _mm_load_ps(m1.data[2].data);
	const __m128 a4 = _mm_load_ps(m1.data[3].data);

	const __m128 b1 = _mm_load_ps(m2.data[0].data);
	const __m128 b2 = _mm_load_ps(m2.data[1].data);
	const __m128 b3 = _mm_load_ps(m2.data[2].data);
	const __m128 b4 = _mm_load_ps(m2.data[3].data);

	return mat(
		vec4(_mm_add_ps(a1, b1)),
		vec4(_mm_add_ps(a2, b2)),
		vec4(_mm_add_ps(a3, b3)),
		vec4(_mm_add_ps(a4, b4))
	);
}

mat gmath::operator-(const mat& m1, const mat& m2) {
	const __m128 a1 = _mm_load_ps(m1.data[0].data);
	const __m128 a2 = _mm_load_ps(m1.data[1].data);
	const __m128 a3 = _mm_load_ps(m1.data[2].data);
	const __m128 a4 = _mm_load_ps(m1.data[3].data);

	const __m128 b1 = _mm_load_ps(m2.data[0].data);
	const __m128 b2 = _mm_load_ps(m2.data[1].data);
	const __m128 b3 = _mm_load_ps(m2.data[2].data);
	const __m128 b4 = _mm_load_ps(m2.data[3].data);

	return mat(
		vec4(_mm_sub_ps(a1, b1)),
		vec4(_mm_sub_ps(a2, b2)),
		vec4(_mm_sub_ps(a3, b3)),
		vec4(_mm_sub_ps(a4, b4))
	);
}