#include <iostream>

#include "ocelot/djinni/cpp/Wave.hpp"
#include "ocelot/djinni/cpp/CoreType.hpp"

#include <cassert>
#include <limits>
#include <optional>
#include <set>

namespace ocelot {

class WaveSingle : public djinni::Wave {
public:
	~WaveSingle() = default;

	void prop() override {
	}
};
}  // namespace ocelot

namespace ocelot::djinni {
std::shared_ptr<Wave> Wave::createWave(CoreType type) {
	switch (type) {
	case CoreType::SINGLE:
		return std::make_shared<ocelot::WaveSingle>();
	default:
		return nullptr;
	}
}
}  // namespace ocelot::djinni
