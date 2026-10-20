// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-cam/src/xml.rs (MIT OR Apache-2.0).
#include "pibridge/cam/xml.h"
#include "pibridge_test.h"

using namespace pibridge::cam;

TEST(nested_attrs_comments_entities) {
    const char* doc = R"(<?xml version="1.0" ?>
<!-- c --><!DOCTYPE x>
<A a="1" b = 'two &amp; &lt;3&gt; &#65;&#x42;'>
  text <![CDATA[ <ignored/> ]]>
  <B x="y"/><!-- inner -->
  <C><D/></C>
  <B x="z"></B>
</A>
)";
    const auto r = *xml::parse(doc);
    CHECK(r.name == "A");
    CHECK(*r.attr("a") == "1");
    CHECK(*r.attr("b") == "two & <3> AB");
    CHECK(r.children.size() == 3);
    const auto bs = r.children_named("B");
    CHECK(bs.size() == 2 && *bs[0]->attr("x") == "y" && *bs[1]->attr("x") == "z");
    CHECK(r.child("C")->children[0].name == "D");
}

TEST(errors) {
    CHECK(!xml::parse(""));
    CHECK(!xml::parse("<A>"));
    CHECK(!xml::parse("<A></B>"));
    CHECK(!xml::parse("<A x=1/>"));
    CHECK(!xml::parse("<A x='&bogus;'/>"));
    CHECK(!xml::parse("<A/><B/>"));
    CHECK(!xml::parse("<A x=\"unterminated/>"));
    // the texts the C ABI reports
    CHECK(xml::parse("").error().text() == "xml: no root element at byte 0");
    CHECK(xml::parse("<A></B>").error().text() == "xml: </B> closes <A> at byte 6");
    CHECK(xml::parse("<!-- x").error().text() == "xml: unterminated construct, expected \"-->\" at byte 0");
}

PIBRIDGE_TEST_MAIN()
