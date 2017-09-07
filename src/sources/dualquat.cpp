#include "..\include\gmath.hpp"

using namespace gmath;

dualquat::dualquat(const quat& real, const quat& dual):
	data(new quat[2]) {
	data[0] = quat(real);
	data[1] = quat(dual);
}

dualquat::dualquat(const vec4& v):
	data(new quat[2]{ quat(1.0f), quat(v) }) {
}

dualquat::dualquat(const quat& r, const vec4& t):
	data(new quat[2]{ r, 0.5f * (quat(t) * r) }) {
}

dualquat::dualquat(const dualquat& other):
	data(new quat[2]) {
	data[0] = quat(other.data[0]);
	data[1] = quat(other.data[1]);
}

dualquat::dualquat(dualquat&& other):
	data(nullptr) {
	this->data = other.data;
	other.data = nullptr;
}

dualquat::~dualquat() {
	if (data != nullptr) {
		delete[] data;
		data = nullptr;
	}
}

dualquat& dualquat::operator=(const dualquat& other) {
	if (this != &other) {
		delete[] data;

		data = new quat[2];
		data[0] = quat(other.data[0]);
		data[1] = quat(other.data[1]);
	}

	return *this;
}

dualquat& dualquat::operator=(dualquat&& other) {
	if (this != &other) {
		delete[] data;

		this->data = other.data;
		other.data = nullptr;
	}

	return *this;
}

quat& dualquat::operator[](const uint32_t i) const {
	return data[i];
}

dualquat dualquat::conjugate() const {
	return dualquat(data[0].conjugate(), data[1].conjugate());
}

dualquat dualquat::dualConjugate() const {
	return dualquat(data[0].conjugate(), -data[1].conjugate());
}

dualquat dualquat::inverse() const {
	const quat pInv = data[0].inverse();
	const quat q = data[1];

	return dualquat(pInv, -1.0f * (pInv * (q * pInv)));
}

vec4 dualquat::transform(const vec4& v) const {
	const dualquat d = v[3] == 0.0f ? dualquat(data[0], quat()) : *this;
	const dualquat result = d * (dualquat(v) * d.dualConjugate());
	return vec4(
		result.data[1].data[1],
		result.data[1].data[2],
		result.data[1].data[3],
		v.data[3]
	);
}

std::string dualquat::toString() const {
	return std::string("non-dual: ") + data[0].toString() + std::string("\n") +
		std::string("dual: ") + data[1].toString();
}

dualquat gmath::operator*(const dualquat& d1, const dualquat& d2) {
	const quat ac = d1.data[0] * d2.data[0];
	const quat ad = d1.data[0] * d2.data[1];
	const quat bc = d1.data[1] * d2.data[0];
	return dualquat(ac, ad + bc);
}

dualquat gmath::operator*(const float& s, const dualquat& d) {
	return dualquat(s * d.data[0], s * d.data[1]);
}

dualquat gmath::operator*(const dualquat& d, const float& s) {
	return dualquat(d.data[0] * s, d.data[1] * s);
}

dualquat gmath::operator+(const dualquat& d1, const dualquat& d2) {
	return dualquat(d1.data[0] + d2.data[0], d1.data[1] + d2.data[1]);
}

dualquat gmath::operator-(const dualquat& d1, const dualquat& d2) {
	return dualquat(d1.data[0] - d2.data[0], d1.data[1] - d2.data[1]);
}