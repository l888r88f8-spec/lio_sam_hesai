#ifndef LIO_SAM_HESAI_COMMON_PARAMETER_ALIAS_H
#define LIO_SAM_HESAI_COMMON_PARAMETER_ALIAS_H

#include <optional>
#include <stdexcept>
#include <string>

// Thrown when a canonical and a legacy key are both set to different values.
// Startup must fail deterministically rather than pick one silently.
class ParameterAliasConflict : public std::runtime_error {
 public:
  explicit ParameterAliasConflict(const std::string& message)
      : std::runtime_error(message) {}
};

template <typename T>
struct ParameterAliasResult {
  T value;
  // True when the resolved value came from the legacy key (either because it
  // was the only one set, or because both were set to the same value). Both
  // cases must emit exactly one deprecation warning per node.
  bool used_legacy{false};
};

// Resolves one canonical/legacy key pair. Precedence rules:
//   1. canonical only            -> canonical value, no warning
//   2. legacy only               -> legacy value, warn once
//   3. both, equal               -> canonical value, warn once
//   4. both, different           -> throw ParameterAliasConflict
//   5. neither                   -> default value, no warning
template <typename T>
ParameterAliasResult<T> ResolveParameterAlias(
    const std::optional<T>& canonical_value,
    const std::optional<T>& legacy_value, const T& default_value) {
  if (canonical_value.has_value() && legacy_value.has_value()) {
    if (!(canonical_value.value() == legacy_value.value())) {
      throw ParameterAliasConflict(
          "canonical and legacy parameters are both set to different values");
    }
    return ParameterAliasResult<T>{canonical_value.value(), true};
  }
  if (canonical_value.has_value()) {
    return ParameterAliasResult<T>{canonical_value.value(), false};
  }
  if (legacy_value.has_value()) {
    return ParameterAliasResult<T>{legacy_value.value(), true};
  }
  return ParameterAliasResult<T>{default_value, false};
}

#endif  // LIO_SAM_HESAI_COMMON_PARAMETER_ALIAS_H
