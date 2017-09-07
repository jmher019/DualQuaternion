#ifndef G_MATH_HPP
#define G_MATH_HPP

#include <xmmintrin.h>

#include <iostream>
#include <math.h>
#include <string>

#define SHUFFLE_PARAM(x, y, z, w) \
	((x) | ((y) << 2) | ((z) << 4) | ((w) << 6))
#define _mm_replicate_x_ps(v) \
	_mm_shuffle_ps((v), (v), _MM_SHUFFLE(0, 0, 0, 0))
#define _mm_replicate_y_ps(v) \
	_mm_shuffle_ps((v), (v), _MM_SHUFFLE(1, 1, 1, 1))
#define _mm_replicate_z_ps(v) \
	_mm_shuffle_ps((v), (v), _MM_SHUFFLE(2, 2, 2, 2))
#define _mm_replicate_w_ps(v) \
	_mm_shuffle_ps((v), (v), _MM_SHUFFLE(3, 3, 3, 3))

namespace gmath {
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//														vec4
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	class vec4 {
	public:
		float* data = nullptr;

		vec4(const float& x = 0.0f, const float& y = 0.0f, const float& z = 0.0f, const float& w = 0.0f);
		vec4(const __m128& data);
		vec4(const vec4& other);
		vec4(vec4&& other);
		~vec4();

		vec4& operator=(const vec4& other);
		vec4& operator=(vec4&& other);
		float& operator[](const uint32_t& i) const;
		vec4 operator-() const;

		float dot(const vec4& other) const;
		float magnitude() const;
		float magnitude2() const;
		vec4 multiply(const vec4& other) const;
		vec4 normalize() const;
		std::string toString() const;

		static float dot(const vec4& v1, const vec4& v2);
		static vec4 multiply(const vec4& v1, const vec4& v2);
	};

	vec4 operator*(const vec4& v1, const vec4& v2);
	vec4 operator*(const float& s, const vec4& v);
	vec4 operator*(const vec4& v, const float& s);
	vec4 operator/(const vec4& q, const float& s);
	vec4 operator+(const vec4& v1, const vec4& v2);
	vec4 operator-(const vec4& v1, const vec4& v2);


	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//														mat
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	class mat {
	public:
		// each entry is a column
		vec4* data = nullptr;

		mat(const vec4& c1 = vec4(1.0f), const vec4& c2 = vec4(0.0f, 1.0f), const vec4& c3 = vec4(0.0f, 0.0f, 1.0f), const vec4& c4 = vec4(0.0f, 0.0f, 0.0f, 1.0f));
		mat(const mat& other);
		mat(mat&& other);
		~mat();

		vec4& operator[](const uint32_t& i) const;
		mat& operator=(const mat& other);
		mat& operator=(mat&& other);

		std::string toString() const;

		static mat translate(const vec4& t);
		static mat rotateX(const float& radians);
		static mat rotateY(const float& radians);
		static mat rotateZ(const float& radians);
		static mat rotate(const vec4& a, const float& radians);
		static mat transform(const vec4& a, const float& radians, const vec4& t);
	};

	vec4 operator*(const mat& m, const vec4& v);
	mat operator*(const mat& m1, const mat& m2);
	mat operator*(const float& s, const mat& m);
	mat operator*(const mat& m, const float& s);
	mat operator/(const mat& m, const float& s);
	mat operator+(const mat& m1, const mat& m2);
	mat operator-(const mat& m1, const mat& m2);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//														quat
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	class quat {
	public:
		// layout of data
		// [0] real component
		// [1] i quaternion unit
		// [2] j quaternion unit
		// [3] k quaternion unit
		float* data = nullptr;

		quat(const float a = 0.0f, const float b = 0.0f, const float c = 0.0f, const float d = 0.0f);
		quat(const __m128& data);
		quat(const vec4& v);
		quat(const quat& other);
		quat(quat&& other);
		~quat();

		float& operator[](const uint32_t& i) const;
		quat& operator=(const quat& other);
		quat& operator=(quat&& other);
		quat operator-() const;

		quat conjugate() const;
		float norm() const;
		quat inverse() const;
		quat normalize() const;
		vec4 transform(const vec4& v) const;
		vec4 transform(const vec4& v, const vec4& t) const;
		std::string toString() const;
	};

	quat operator*(const quat& q1, const quat& q2);
	quat operator*(const float& s, const quat& q);
	quat operator*(const quat& q, const float& s);
	quat operator/(const quat& q, const float& s);
	quat operator+(const quat& v1, const quat& v2);
	quat operator-(const quat& v1, const quat& v2);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//														dualquat
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	class dualquat {
	public:
		quat* data;

		dualquat(const quat& real = quat(), const quat& dual = quat());
		dualquat(const vec4& v);
		dualquat(const quat& r, const vec4& t);
		dualquat(const dualquat& other);
		dualquat(dualquat&& other);
		~dualquat();

		dualquat& operator=(const dualquat& other);
		dualquat& operator=(dualquat&& other);
		quat& operator[](const uint32_t i) const;

		dualquat conjugate() const;
		dualquat dualConjugate() const;
		dualquat inverse() const;
		vec4 transform(const vec4& v) const;
		std::string toString() const;
	};

	dualquat operator*(const dualquat& d1, const dualquat& d2);
	dualquat operator*(const float& s, const dualquat& d);
	dualquat operator*(const dualquat& d, const float& s);
	dualquat operator+(const dualquat& d1, const dualquat& d2);
	dualquat operator-(const dualquat& d1, const dualquat& d2);
}

#endif // !G_MATH_HPP
