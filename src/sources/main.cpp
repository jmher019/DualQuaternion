#include <iostream>
#include <chrono>
#include <functional>

#include "..\include\gmath.hpp"

using namespace gmath;

const float PI = 3.1415927f;
const int numTrials = 50000;

// For this test:
// concatenate transformations of this order:
// translate 3, 4, 5
// rotate 30 deg about y axis
// rotate 20 deg about z axis
// rotate 25 deg about x axis
// translate -7, -9, -3
// rotate 99 deg about 1/sqrt(2), 1/sqrt(2), 0 axis
// translate 0, 4, -1
// rotate 12 deg about -1/sqrt(3), -1/sqrt(3), 1/sqrt(3) axis
void testConcatTransformsMatrix() {
	const float radians1 = 30.0f * PI / 180.0f;
	mat m = mat::translate(vec4(3.0f, 4.0f, 5.0f));
	m = mat::rotateY(radians1) * m;

	const float radians2 = 20.0f * PI / 180.0f;
	m = mat::rotateZ(radians2) * m;

	const float radians3 = 25.0f * PI / 180.0f;
	m = mat::rotateX(radians3) * m;
	m = mat::translate(vec4(-7.0f, -9.0f, -3.0f)) * m;

	const float radians4 = 99.0f * PI / 180.0f;
	const float c = 1.0f / sqrtf(2.0f);
	const vec4 a(c, c);
	m = mat::transform(a, radians4, vec4(0.0f, 4.0f, -1.0f)) * m;

	const float radians5 = 12.0f * PI / 180.0f;
	const float d = 1.0f / sqrtf(3.0f);
	const vec4 b(-d, -d, d);
	m = mat::rotate(b, radians5) * m;

	//std::cout << m.toString() << std::endl;
}

// For this test:
// concatenate transformations of this order:
// translate 3, 4, 5
// rotate 30 deg about y axis
// rotate 20 deg about z axis
// rotate 25 deg about x axis
// translate -7, -9, -3
// rotate 99 deg about 1/sqrt(2), 1/sqrt(2), 0 axis
// translate 0, 4, -1
// rotate 12 deg about -1/sqrt(3), -1/sqrt(3), 1/sqrt(3) axis
// construct resulting matrix
void testConcatTransformQuatAndVec() {
	quat q;
	vec4 t = vec4(3.0f, 4.0f, 5.0f);

	const float radians1 = 30.0f * PI / 180.0f;
	const float COS1 = cosf(radians1 / 2.0f);
	const float SIN1 = sinf(radians1 / 2.0f);
	q = quat(COS1, 0.0f, SIN1, 0.0f);
	// update t
	t = q.transform(t);

	const float radians2 = 20.0f * PI / 180.0f;
	const float COS2 = cosf(radians2 / 2.0f);
	const float SIN2 = sinf(radians2 / 2.0f);
	const quat q1(COS2, 0.0f, 0.0f, SIN2);
	q = q1 * q;
	// update t
	t = q1.transform(t);

	const float radians3 = 25.0f * PI / 180.0f;
	const float COS3 = cosf(radians3 / 2.0f);
	const float SIN3 = sinf(radians3 / 2.0f);
	const quat q2(COS3, SIN3, 0.0f, 0.0f);
	q = q2 * q;
	// update t
	t = q2.transform(t);

	t = t + vec4(-7.0f, -9.0f, -3.0f);

	const float radians4 = 99.0f * PI / 180.0f;
	const float COS4 = cosf(radians4 / 2.0f);
	const float SIN4 = sinf(radians4 / 2.0f);
	const float c = SIN4 / sqrtf(2.0f);
	const quat q3(COS4, c, c, 0.0f);
	q = q3 * q;
	// update t
	t = q3.transform(t);

	t = t + vec4(0.0f, 4.0f, -1.0f);

	const float radians5 = 12.0f * PI / 180.0f;
	const float COS5 = cosf(radians5 / 2.0f);
	const float SIN5 = sinf(radians5 / 2.0f);
	const float d = SIN5 / sqrtf(3.0f);
	const quat q4(COS5, -d, -d, d);
	q = q4 * q;
	// update t
	t = q4.transform(t);

	// construct matrix
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

	const mat result(
		vec4(1.0f - 2.0f * (mul2[2] + mul3[3]), 2.0f * (mul1[2] + mul3[0]), 2.0f * (mul1[3] - mul2[0])),
		vec4(2.0f * (mul1[2] - mul3[0]), 1.0f - 2.0f * (mul1[1] + mul3[3]), 2.0f * (mul2[3] + mul1[0])),
		vec4(2.0f * (mul1[3] + mul2[0]), 2.0f * (mul2[3] - mul1[0]), 1.0f - 2.0f * (mul1[1] + mul2[2])),
		vec4(t[0], t[1], t[2], 1.0f)
	);

	//std::cout << result.toString() << std::endl;
}

// For this test:
// concatenate transformations of this order:
// translate 3, 4, 5
// rotate 30 deg about y axis
// rotate 20 deg about z axis
// rotate 25 deg about x axis
// translate -7, -9, -3
// rotate 99 deg about 1/sqrt(2), 1/sqrt(2), 0 axis
// translate 0, 4, -1
// rotate 12 deg about -1/sqrt(3), -1/sqrt(3), 1/sqrt(3) axis
// construct resulting matrix
void testConcatTransformDualQuat() {
	dualquat d(quat(1.0f), vec4(3.0f, 4.0f, 5.0f));

	const float radians1 = 30.0f * PI / 180.0f;
	const float COS1 = cosf(radians1 / 2.0f);
	const float SIN1 = sinf(radians1 / 2.0f);
	d = dualquat(quat(COS1, 0.0f, SIN1, 0.0f)) * d;

	const float radians2 = 20.0f * PI / 180.0f;
	const float COS2 = cosf(radians2 / 2.0f);
	const float SIN2 = sinf(radians2 / 2.0f);
	d = dualquat(quat(COS2, 0.0f, 0.0f, SIN2)) * d;

	const float radians3 = 25.0f * PI / 180.0f;
	const float COS3 = cosf(radians3 / 2.0f);
	const float SIN3 = sinf(radians3 / 2.0f);
	d = dualquat(quat(COS3, SIN3, 0.0f, 0.0f)) * d;
	
	d = dualquat(quat(1.0f), vec4(-7.0f, -9.0f, -3.0f)) * d;
	
	const float radians4 = 99.0f * PI / 180.0f;
	const float COS4 = cosf(radians4 / 2.0f);
	const float SIN4 = sinf(radians4 / 2.0f);
	const float c = SIN4 / sqrtf(2.0f);
	d = dualquat(quat(COS4, c, c, 0.0f)) * d;

	d = dualquat(quat(1.0f), vec4(0.0f, 4.0f, -1.0f)) * d;

	const float radians5 = 12.0f * PI / 180.0f;
	const float COS5 = cosf(radians5 / 2.0f);
	const float SIN5 = sinf(radians5 / 2.0f);
	const float e = SIN5 / sqrtf(3.0f);
	d = dualquat(quat(COS5, -e, -e, e)) * d;

	// construct matrix
	quat t = 2.0f * (d[1] * d[0].conjugate());

	__m128 wxyz = _mm_load_ps(d[0].data);

	__m128 wwww = _mm_replicate_x_ps(wxyz);
	__m128 xxxx = _mm_replicate_y_ps(wxyz);
	__m128 yyyy = _mm_replicate_z_ps(wxyz);
	__m128 zzzz = _mm_replicate_w_ps(wxyz);

	// contains ww, wx, wy, wz
	float prod1[4];
	_mm_store_ps(&prod1[0], _mm_mul_ps(wwww, wxyz));
	
	// contains xw, xx, xy, xz
	float prod2[4];
	_mm_store_ps(&prod2[0], _mm_mul_ps(xxxx, wxyz));

	// contains yw, yx, yy, yz
	float prod3[4];
	_mm_store_ps(&prod3[0], _mm_mul_ps(yyyy, wxyz));


	// contains zw, zx, zy, zz
	float prod4[4];
	_mm_store_ps(&prod4[0], _mm_mul_ps(zzzz, wxyz));

	const mat result(
		vec4(
			prod1[0] + prod2[1] - prod3[2] - prod4[3],
			2.0f * (prod2[2] + prod1[3]),
			2.0f * (prod2[3] - prod1[2])
		),
		vec4(
			2.0f * (prod2[2] - prod1[3]),
			prod1[0] - prod2[1] + prod3[2] - prod4[3],
			2.0f * (prod3[3] + prod1[1])
		),
		vec4(
			2.0f * (prod2[3] + prod1[2]),
			2.0f * (prod3[3] - prod1[1]),
			prod1[0] - prod2[1] - prod3[2] + prod4[3]
		),
		vec4(t[1], t[2], t[3], 1.0f)
	);

	//std::cout << result.toString() << std::endl;
}

void runTest(const std::function<void()>& f, int numTrials) {
	std::chrono::time_point<std::chrono::system_clock> t1;
	std::chrono::time_point<std::chrono::system_clock> t2;
	double totalTime = 0;

	for (int i = 0; i < numTrials; i++) {
		t1 = std::chrono::system_clock::now();

		f();

		t2 = std::chrono::system_clock::now();
		std::chrono::duration<double> timeElapsed = t2 - t1;
		totalTime += timeElapsed.count();
	}

	std::cout << "Finished running " << numTrials << " trials. Time ellapsed: " << totalTime << "s." << std::endl;
}

int main() {
	std::cout << "Concatenating Matrix transforms test: " << std::endl;
	runTest(testConcatTransformsMatrix, numTrials);
	std::cout << "Concatenating quaternion and translation vectors test: " << std::endl;
	runTest(testConcatTransformQuatAndVec, numTrials);
	std::cout << "Concatenating dual quaternions test: " << std::endl;
	runTest(testConcatTransformDualQuat, numTrials);
	std::system("pause");
	return 1;
}