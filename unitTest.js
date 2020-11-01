function UnitTest(actual, expected, context) {
  if (context == undefined) {
    context = "expected";
  }
  if (typeof expected != typeof actual) {
    throw Error([context, "actual and expected are different types (" + (typeof actual) + " != " + (typeof expected) + ")"].join(": "));
  }
  if (typeof expected == "object") {
    if (expected instanceof Array) {
      if (actual.length != expected.length) {
        throw Error([context, "actual array length does not match expected (" + actual.length + " != " + expected.length + ")"].join(": "));
      }
      actual.forEach(function(v, i) {
        UnitTest(v, expected[i], context + "[" + i + "]");
      });
    } else {
      for (const k in expected) {
        if (!(k in actual)) {
          throw Error([context, "property '" + k + "' is missing on actual value (expected " + expected[k] + ")"].join(": "));
        } else {
          UnitTest(actual[k], expected[k], context + "." + k);
        }
      }
      for (const k in actual) {
        if (!(k in expected)) {
          throw Error([context, "property '" + k + "' is unexpectedly present on actual value"].join(": "));
        } else {
          UnitTest(actual[k], expected[k], context + "." + k);
        }
      }
    }
  } else if (actual !== expected) {
    throw Error([context, "actual value does not match expected (" + actual + " != " + expected + ")"].join(": "));
  }
}

module.exports = UnitTest;

if (require.main == module) {
  // unit-test the UnitTest function, of course
  UnitTest(true, true);
  try{UnitTest(true, false);}catch(e){UnitTest(e.message, "expected: actual value does not match expected (true != false)");}
  try{UnitTest(true, 1);}catch(e){UnitTest(e.message, "expected: actual and expected are different types (boolean != number)");}
  try{UnitTest(false, []);}catch(e){UnitTest(e.message, "expected: actual and expected are different types (boolean != object)");}
  try{UnitTest(false, "");}catch(e){UnitTest(e.message, "expected: actual and expected are different types (boolean != string)");}
  try{UnitTest(false, undefined);}catch(e){UnitTest(e.message, "expected: actual and expected are different types (boolean != undefined)");}

  UnitTest([0, 1, 2], [0, 1, 2]);
  try{UnitTest([0, 1, 2], [0, 1]);}catch(e){UnitTest(e.message, "expected: actual array length does not match expected (3 != 2)");}
  try{UnitTest([0, 1, 2], [0, 1, 3]);}catch(e){UnitTest(e.message, "expected[2]: actual value does not match expected (2 != 3)");}
  
  UnitTest({a: 5, b: 6}, {a: 5, b: 6});
  try{UnitTest({a: 5}, {a: 5, b: 6});}catch(e){UnitTest(e.message, "expected: property 'b' is missing on actual value (expected 6)");}
  try{UnitTest({a: 5, b: 6}, {a: 5});}catch(e){UnitTest(e.message, "expected: property 'b' is unexpectedly present on actual value");}
}
