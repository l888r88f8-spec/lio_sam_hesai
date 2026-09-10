#include "common/parameter_alias.h"

#include <gtest/gtest.h>

#include <optional>
#include <string>
#include <vector>

TEST(ParameterAlias, UsesCanonical) {
  EXPECT_EQ(ResolveParameterAlias<int>(3, std::nullopt, 1).value, 3);
}

TEST(ParameterAlias, CanonicalAloneIsNotMarkedLegacy) {
  const auto result = ResolveParameterAlias<int>(3, std::nullopt, 1);
  EXPECT_EQ(result.value, 3);
  EXPECT_FALSE(result.used_legacy);
}

TEST(ParameterAlias, UsesLegacyAndMarksIt) {
  const auto result = ResolveParameterAlias<int>(std::nullopt, 4, 1);
  EXPECT_EQ(result.value, 4);
  EXPECT_TRUE(result.used_legacy);
}

TEST(ParameterAlias, UsesDefaultWhenNeitherIsSet) {
  const auto result = ResolveParameterAlias<int>(std::nullopt, std::nullopt, 7);
  EXPECT_EQ(result.value, 7);
  EXPECT_FALSE(result.used_legacy);
}

TEST(ParameterAlias, EqualDualValuesPreferCanonicalAndMarkLegacy) {
  const auto result = ResolveParameterAlias<int>(3, 3, 1);
  EXPECT_EQ(result.value, 3);
  EXPECT_TRUE(result.used_legacy);
}

TEST(ParameterAlias, RejectsConflict) {
  EXPECT_THROW(ResolveParameterAlias<int>(3, 4, 1), ParameterAliasConflict);
}

TEST(ParameterAlias, ConflictMessageIsRecoverable) {
  try {
    ResolveParameterAlias<int>(3, 4, 1);
    FAIL() << "expected ParameterAliasConflict";
  } catch (const ParameterAliasConflict& error) {
    EXPECT_FALSE(std::string(error.what()).empty());
  }
}

TEST(ParameterAlias, ComparesStrings) {
  EXPECT_EQ(ResolveParameterAlias<std::string>("canonical", std::nullopt,
                                               "default")
                .value,
            "canonical");
  const auto legacy =
      ResolveParameterAlias<std::string>(std::nullopt, "legacy", "default");
  EXPECT_EQ(legacy.value, "legacy");
  EXPECT_TRUE(legacy.used_legacy);
  EXPECT_THROW(
      ResolveParameterAlias<std::string>("a", "b", "default"),
      ParameterAliasConflict);
}

TEST(ParameterAlias, ComparesVectorsElementWiseExactly) {
  const std::vector<double> canonical = {1.0, 2.0, 3.0};
  const std::vector<double> legacy = {1.0, 2.0, 3.0};
  const std::vector<double> other = {1.0, 2.0, 3.5};

  const auto equal = ResolveParameterAlias<std::vector<double>>(
      canonical, legacy, std::vector<double>{});
  EXPECT_EQ(equal.value, canonical);
  EXPECT_TRUE(equal.used_legacy);

  EXPECT_THROW(
      ResolveParameterAlias<std::vector<double>>(canonical, other,
                                                 std::vector<double>{}),
      ParameterAliasConflict);
}

TEST(ParameterAlias, ComparesDoublesExactly) {
  EXPECT_THROW(ResolveParameterAlias<double>(1.0, 1.0000001, 0.0),
               ParameterAliasConflict);
  const auto equal = ResolveParameterAlias<double>(2.5, 2.5, 0.0);
  EXPECT_DOUBLE_EQ(equal.value, 2.5);
}

TEST(ParameterAlias, CompareFailureSurfacesAsRuntimeError) {
  // ParameterAliasConflict must be catchable as std::runtime_error so callers
  // that only know the standard hierarchy still fail deterministically.
  EXPECT_THROW(ResolveParameterAlias<int>(1, 2, 0), std::runtime_error);
}
