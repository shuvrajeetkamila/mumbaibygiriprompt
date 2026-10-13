// =============================================================================
//  MUMBAI 1:1 EXACT — tests/test_json.cpp
//  The in-tree JSON reader/writer that loads every authored and source control
//  file (configs, terrain control points, corridor definitions, manifests).
// =============================================================================
#include "engine/core/json.h"
#include "tests/test_harness.h"

using namespace mumbai;

TEST(jsonParsesNestedDocuments) {
    const char* text = R"({
      "name": "ward A", "count": 24, "scale": 1.5e3, "flag": true, "none": null,
      "list": [1, 2, 3, {"deep": "value"}],
      "escaped": "line\nbreak \"quoted\" \\ slash",
      "nested": { "a": { "b": { "c": 42 } } }
    })";
    Json j;
    std::string err;
    CHECK(Json::parse(text, j, &err));
    CHECK_EQ(j["name"].asString(), std::string("ward A"));
    CHECK_EQ(j["count"].asInt(), 24);
    CHECK_NEAR(j["scale"].asNumber(), 1500.0, 1e-9);
    CHECK(j["flag"].asBool());
    CHECK(j["none"].isNull());
    CHECK_EQ(j["list"].size(), 4u);
    CHECK_EQ(j["list"][3]["deep"].asString(), std::string("value"));
    CHECK_EQ(j["nested"]["a"]["b"]["c"].asInt(), 42);
    // Missing keys return null rather than throwing: every loader relies on it.
    // (The non-const operator[] inserts, exactly like std::map, so `has` is
    // checked first.)
    CHECK_FALSE(j.has("absent"));
    CHECK(j["absent"].isNull());
}

TEST(jsonRoundTripsThroughDump) {
    Json root = Json::object();
    root.set("name", Json(std::string("Mumbai")));
    root.set("cells", Json(924.0));
    Json arr = Json::array();
    for (int i = 0; i < 3; ++i) {
        Json item = Json::object();
        item.set("i", Json(static_cast<f64>(i)));
        item.set("lon", Json(72.9 + i * 0.01));
        arr.push(item);
    }
    root.set("points", arr);

    Json back;
    std::string err;
    CHECK(Json::parse(root.dump(), back, &err));
    CHECK_EQ(back["name"].asString(), std::string("Mumbai"));
    CHECK_EQ(back["points"].size(), 3u);
    CHECK_NEAR(back["points"][2]["lon"].asNumber(), 72.92, 1e-9);
    CHECK_EQ(back["cells"].asInt(), 924);
}

TEST(jsonRejectsMalformedInput) {
    Json j;
    std::string err;
    CHECK_FALSE(Json::parse("{\"a\": }", j, &err));
    CHECK_FALSE(err.empty());
    CHECK_FALSE(Json::parse("[1, 2", j, &err));
    CHECK_FALSE(Json::parse("", j, &err));
}
