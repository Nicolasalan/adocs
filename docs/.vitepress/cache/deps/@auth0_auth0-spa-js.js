// node_modules/@auth0/auth0-spa-js/dist/auth0-spa-js.production.esm.js
var e = function(t2, n2) {
  return e = Object.setPrototypeOf || { __proto__: [] } instanceof Array && function(e2, t3) {
    e2.__proto__ = t3;
  } || function(e2, t3) {
    for (var n3 in t3)
      Object.prototype.hasOwnProperty.call(t3, n3) && (e2[n3] = t3[n3]);
  }, e(t2, n2);
};
function t(t2, n2) {
  if ("function" != typeof n2 && null !== n2)
    throw new TypeError("Class extends value " + String(n2) + " is not a constructor or null");
  function r2() {
    this.constructor = t2;
  }
  e(t2, n2), t2.prototype = null === n2 ? Object.create(n2) : (r2.prototype = n2.prototype, new r2());
}
var n = function() {
  return n = Object.assign || function(e2) {
    for (var t2, n2 = 1, r2 = arguments.length; n2 < r2; n2++)
      for (var o2 in t2 = arguments[n2])
        Object.prototype.hasOwnProperty.call(t2, o2) && (e2[o2] = t2[o2]);
    return e2;
  }, n.apply(this, arguments);
};
function r(e2, t2) {
  var n2 = {};
  for (var r2 in e2)
    Object.prototype.hasOwnProperty.call(e2, r2) && t2.indexOf(r2) < 0 && (n2[r2] = e2[r2]);
  if (null != e2 && "function" == typeof Object.getOwnPropertySymbols) {
    var o2 = 0;
    for (r2 = Object.getOwnPropertySymbols(e2); o2 < r2.length; o2++)
      t2.indexOf(r2[o2]) < 0 && Object.prototype.propertyIsEnumerable.call(e2, r2[o2]) && (n2[r2[o2]] = e2[r2[o2]]);
  }
  return n2;
}
function o(e2, t2, n2, r2) {
  return new (n2 || (n2 = Promise))(function(o2, i2) {
    function c2(e3) {
      try {
        s2(r2.next(e3));
      } catch (e4) {
        i2(e4);
      }
    }
    function a2(e3) {
      try {
        s2(r2.throw(e3));
      } catch (e4) {
        i2(e4);
      }
    }
    function s2(e3) {
      var t3;
      e3.done ? o2(e3.value) : (t3 = e3.value, t3 instanceof n2 ? t3 : new n2(function(e4) {
        e4(t3);
      })).then(c2, a2);
    }
    s2((r2 = r2.apply(e2, t2 || [])).next());
  });
}
function i(e2, t2) {
  var n2, r2, o2, i2, c2 = { label: 0, sent: function() {
    if (1 & o2[0])
      throw o2[1];
    return o2[1];
  }, trys: [], ops: [] };
  return i2 = { next: a2(0), throw: a2(1), return: a2(2) }, "function" == typeof Symbol && (i2[Symbol.iterator] = function() {
    return this;
  }), i2;
  function a2(i3) {
    return function(a3) {
      return function(i4) {
        if (n2)
          throw new TypeError("Generator is already executing.");
        for (; c2; )
          try {
            if (n2 = 1, r2 && (o2 = 2 & i4[0] ? r2.return : i4[0] ? r2.throw || ((o2 = r2.return) && o2.call(r2), 0) : r2.next) && !(o2 = o2.call(r2, i4[1])).done)
              return o2;
            switch (r2 = 0, o2 && (i4 = [2 & i4[0], o2.value]), i4[0]) {
              case 0:
              case 1:
                o2 = i4;
                break;
              case 4:
                return c2.label++, { value: i4[1], done: false };
              case 5:
                c2.label++, r2 = i4[1], i4 = [0];
                continue;
              case 7:
                i4 = c2.ops.pop(), c2.trys.pop();
                continue;
              default:
                if (!(o2 = c2.trys, (o2 = o2.length > 0 && o2[o2.length - 1]) || 6 !== i4[0] && 2 !== i4[0])) {
                  c2 = 0;
                  continue;
                }
                if (3 === i4[0] && (!o2 || i4[1] > o2[0] && i4[1] < o2[3])) {
                  c2.label = i4[1];
                  break;
                }
                if (6 === i4[0] && c2.label < o2[1]) {
                  c2.label = o2[1], o2 = i4;
                  break;
                }
                if (o2 && c2.label < o2[2]) {
                  c2.label = o2[2], c2.ops.push(i4);
                  break;
                }
                o2[2] && c2.ops.pop(), c2.trys.pop();
                continue;
            }
            i4 = t2.call(e2, c2);
          } catch (e3) {
            i4 = [6, e3], r2 = 0;
          } finally {
            n2 = o2 = 0;
          }
        if (5 & i4[0])
          throw i4[1];
        return { value: i4[0] ? i4[1] : void 0, done: true };
      }([i3, a3]);
    };
  }
}
function c(e2, t2) {
  var n2 = "function" == typeof Symbol && e2[Symbol.iterator];
  if (!n2)
    return e2;
  var r2, o2, i2 = n2.call(e2), c2 = [];
  try {
    for (; (void 0 === t2 || t2-- > 0) && !(r2 = i2.next()).done; )
      c2.push(r2.value);
  } catch (e3) {
    o2 = { error: e3 };
  } finally {
    try {
      r2 && !r2.done && (n2 = i2.return) && n2.call(i2);
    } finally {
      if (o2)
        throw o2.error;
    }
  }
  return c2;
}
function a(e2, t2, n2) {
  if (n2 || 2 === arguments.length)
    for (var r2, o2 = 0, i2 = t2.length; o2 < i2; o2++)
      !r2 && o2 in t2 || (r2 || (r2 = Array.prototype.slice.call(t2, 0, o2)), r2[o2] = t2[o2]);
  return e2.concat(r2 || Array.prototype.slice.call(t2));
}
var s = "undefined" != typeof globalThis ? globalThis : "undefined" != typeof window ? window : "undefined" != typeof global ? global : "undefined" != typeof self ? self : {};
function u(e2) {
  return e2 && e2.__esModule && Object.prototype.hasOwnProperty.call(e2, "default") ? e2.default : e2;
}
function l(e2, t2) {
  return e2(t2 = { exports: {} }, t2.exports), t2.exports;
}
var f;
var d;
var h = function(e2) {
  return e2 && e2.Math == Math && e2;
};
var p = h("object" == typeof globalThis && globalThis) || h("object" == typeof window && window) || h("object" == typeof self && self) || h("object" == typeof s && s) || function() {
  return this;
}() || Function("return this")();
var y = function(e2) {
  try {
    return !!e2();
  } catch (e3) {
    return true;
  }
};
var v = !y(function() {
  return 7 != Object.defineProperty({}, 1, { get: function() {
    return 7;
  } })[1];
});
var m = !y(function() {
  var e2 = function() {
  }.bind();
  return "function" != typeof e2 || e2.hasOwnProperty("prototype");
});
var b = Function.prototype.call;
var g = m ? b.bind(b) : function() {
  return b.apply(b, arguments);
};
var w = {}.propertyIsEnumerable;
var S = Object.getOwnPropertyDescriptor;
var k = S && !w.call({ 1: 2 }, 1) ? function(e2) {
  var t2 = S(this, e2);
  return !!t2 && t2.enumerable;
} : w;
var _ = { f: k };
var I = function(e2, t2) {
  return { enumerable: !(1 & e2), configurable: !(2 & e2), writable: !(4 & e2), value: t2 };
};
var O = Function;
var x = O.prototype;
var T = x.bind;
var C = x.call;
var j = m && T.bind(C, C);
var L = function(e2) {
  return e2 instanceof O ? m ? j(e2) : function() {
    return C.apply(e2, arguments);
  } : void 0;
};
var R = L({}.toString);
var W = L("".slice);
var Z = function(e2) {
  return W(R(e2), 8, -1);
};
var E = Object;
var G = L("".split);
var A = y(function() {
  return !E("z").propertyIsEnumerable(0);
}) ? function(e2) {
  return "String" == Z(e2) ? G(e2, "") : E(e2);
} : E;
var P = function(e2) {
  return null == e2;
};
var X = TypeError;
var F = function(e2) {
  if (P(e2))
    throw X("Can't call method on " + e2);
  return e2;
};
var K = function(e2) {
  return A(F(e2));
};
var N = "object" == typeof document && document.all;
var U = { all: N, IS_HTMLDDA: void 0 === N && void 0 !== N };
var D = U.all;
var H = U.IS_HTMLDDA ? function(e2) {
  return "function" == typeof e2 || e2 === D;
} : function(e2) {
  return "function" == typeof e2;
};
var Y = U.all;
var J = U.IS_HTMLDDA ? function(e2) {
  return "object" == typeof e2 ? null !== e2 : H(e2) || e2 === Y;
} : function(e2) {
  return "object" == typeof e2 ? null !== e2 : H(e2);
};
var V = function(e2) {
  return H(e2) ? e2 : void 0;
};
var z = function(e2, t2) {
  return arguments.length < 2 ? V(p[e2]) : p[e2] && p[e2][t2];
};
var M = L({}.isPrototypeOf);
var B = z("navigator", "userAgent") || "";
var Q = p.process;
var q = p.Deno;
var $ = Q && Q.versions || q && q.version;
var ee = $ && $.v8;
ee && (d = (f = ee.split("."))[0] > 0 && f[0] < 4 ? 1 : +(f[0] + f[1])), !d && B && (!(f = B.match(/Edge\/(\d+)/)) || f[1] >= 74) && (f = B.match(/Chrome\/(\d+)/)) && (d = +f[1]);
var te = d;
var ne = !!Object.getOwnPropertySymbols && !y(function() {
  var e2 = Symbol();
  return !String(e2) || !(Object(e2) instanceof Symbol) || !Symbol.sham && te && te < 41;
});
var re = ne && !Symbol.sham && "symbol" == typeof Symbol.iterator;
var oe = Object;
var ie = re ? function(e2) {
  return "symbol" == typeof e2;
} : function(e2) {
  var t2 = z("Symbol");
  return H(t2) && M(t2.prototype, oe(e2));
};
var ce = String;
var ae = function(e2) {
  try {
    return ce(e2);
  } catch (e3) {
    return "Object";
  }
};
var se = TypeError;
var ue = function(e2) {
  if (H(e2))
    return e2;
  throw se(ae(e2) + " is not a function");
};
var le = function(e2, t2) {
  var n2 = e2[t2];
  return P(n2) ? void 0 : ue(n2);
};
var fe = TypeError;
var de = Object.defineProperty;
var he = function(e2, t2) {
  try {
    de(p, e2, { value: t2, configurable: true, writable: true });
  } catch (n2) {
    p[e2] = t2;
  }
  return t2;
};
var pe = p["__core-js_shared__"] || he("__core-js_shared__", {});
var ye = l(function(e2) {
  (e2.exports = function(e3, t2) {
    return pe[e3] || (pe[e3] = void 0 !== t2 ? t2 : {});
  })("versions", []).push({ version: "3.25.4", mode: "global", copyright: "© 2014-2022 Denis Pushkarev (zloirock.ru)", license: "https://github.com/zloirock/core-js/blob/v3.25.4/LICENSE", source: "https://github.com/zloirock/core-js" });
});
var ve = Object;
var me = function(e2) {
  return ve(F(e2));
};
var be = L({}.hasOwnProperty);
var ge = Object.hasOwn || function(e2, t2) {
  return be(me(e2), t2);
};
var we = 0;
var Se = Math.random();
var ke = L(1 .toString);
var _e = function(e2) {
  return "Symbol(" + (void 0 === e2 ? "" : e2) + ")_" + ke(++we + Se, 36);
};
var Ie = ye("wks");
var Oe = p.Symbol;
var xe = Oe && Oe.for;
var Te = re ? Oe : Oe && Oe.withoutSetter || _e;
var Ce = function(e2) {
  if (!ge(Ie, e2) || !ne && "string" != typeof Ie[e2]) {
    var t2 = "Symbol." + e2;
    ne && ge(Oe, e2) ? Ie[e2] = Oe[e2] : Ie[e2] = re && xe ? xe(t2) : Te(t2);
  }
  return Ie[e2];
};
var je = TypeError;
var Le = Ce("toPrimitive");
var Re = function(e2, t2) {
  if (!J(e2) || ie(e2))
    return e2;
  var n2, r2 = le(e2, Le);
  if (r2) {
    if (void 0 === t2 && (t2 = "default"), n2 = g(r2, e2, t2), !J(n2) || ie(n2))
      return n2;
    throw je("Can't convert object to primitive value");
  }
  return void 0 === t2 && (t2 = "number"), function(e3, t3) {
    var n3, r3;
    if ("string" === t3 && H(n3 = e3.toString) && !J(r3 = g(n3, e3)))
      return r3;
    if (H(n3 = e3.valueOf) && !J(r3 = g(n3, e3)))
      return r3;
    if ("string" !== t3 && H(n3 = e3.toString) && !J(r3 = g(n3, e3)))
      return r3;
    throw fe("Can't convert object to primitive value");
  }(e2, t2);
};
var We = function(e2) {
  var t2 = Re(e2, "string");
  return ie(t2) ? t2 : t2 + "";
};
var Ze = p.document;
var Ee = J(Ze) && J(Ze.createElement);
var Ge = function(e2) {
  return Ee ? Ze.createElement(e2) : {};
};
var Ae = !v && !y(function() {
  return 7 != Object.defineProperty(Ge("div"), "a", { get: function() {
    return 7;
  } }).a;
});
var Pe = Object.getOwnPropertyDescriptor;
var Xe = { f: v ? Pe : function(e2, t2) {
  if (e2 = K(e2), t2 = We(t2), Ae)
    try {
      return Pe(e2, t2);
    } catch (e3) {
    }
  if (ge(e2, t2))
    return I(!g(_.f, e2, t2), e2[t2]);
} };
var Fe = v && y(function() {
  return 42 != Object.defineProperty(function() {
  }, "prototype", { value: 42, writable: false }).prototype;
});
var Ke = String;
var Ne = TypeError;
var Ue = function(e2) {
  if (J(e2))
    return e2;
  throw Ne(Ke(e2) + " is not an object");
};
var De = TypeError;
var He = Object.defineProperty;
var Ye = Object.getOwnPropertyDescriptor;
var Je = { f: v ? Fe ? function(e2, t2, n2) {
  if (Ue(e2), t2 = We(t2), Ue(n2), "function" == typeof e2 && "prototype" === t2 && "value" in n2 && "writable" in n2 && !n2.writable) {
    var r2 = Ye(e2, t2);
    r2 && r2.writable && (e2[t2] = n2.value, n2 = { configurable: "configurable" in n2 ? n2.configurable : r2.configurable, enumerable: "enumerable" in n2 ? n2.enumerable : r2.enumerable, writable: false });
  }
  return He(e2, t2, n2);
} : He : function(e2, t2, n2) {
  if (Ue(e2), t2 = We(t2), Ue(n2), Ae)
    try {
      return He(e2, t2, n2);
    } catch (e3) {
    }
  if ("get" in n2 || "set" in n2)
    throw De("Accessors not supported");
  return "value" in n2 && (e2[t2] = n2.value), e2;
} };
var Ve = v ? function(e2, t2, n2) {
  return Je.f(e2, t2, I(1, n2));
} : function(e2, t2, n2) {
  return e2[t2] = n2, e2;
};
var ze = Function.prototype;
var Me = v && Object.getOwnPropertyDescriptor;
var Be = ge(ze, "name");
var Qe = { EXISTS: Be, PROPER: Be && "something" === function() {
}.name, CONFIGURABLE: Be && (!v || v && Me(ze, "name").configurable) };
var qe = L(Function.toString);
H(pe.inspectSource) || (pe.inspectSource = function(e2) {
  return qe(e2);
});
var $e;
var et;
var tt;
var nt = pe.inspectSource;
var rt = p.WeakMap;
var ot = H(rt) && /native code/.test(String(rt));
var it = ye("keys");
var ct = function(e2) {
  return it[e2] || (it[e2] = _e(e2));
};
var at = {};
var st = p.TypeError;
var ut = p.WeakMap;
if (ot || pe.state) {
  lt = pe.state || (pe.state = new ut()), ft = L(lt.get), dt = L(lt.has), ht = L(lt.set);
  $e = function(e2, t2) {
    if (dt(lt, e2))
      throw st("Object already initialized");
    return t2.facade = e2, ht(lt, e2, t2), t2;
  }, et = function(e2) {
    return ft(lt, e2) || {};
  }, tt = function(e2) {
    return dt(lt, e2);
  };
} else {
  pt = ct("state");
  at[pt] = true, $e = function(e2, t2) {
    if (ge(e2, pt))
      throw st("Object already initialized");
    return t2.facade = e2, Ve(e2, pt, t2), t2;
  }, et = function(e2) {
    return ge(e2, pt) ? e2[pt] : {};
  }, tt = function(e2) {
    return ge(e2, pt);
  };
}
var lt;
var ft;
var dt;
var ht;
var pt;
var yt = { set: $e, get: et, has: tt, enforce: function(e2) {
  return tt(e2) ? et(e2) : $e(e2, {});
}, getterFor: function(e2) {
  return function(t2) {
    var n2;
    if (!J(t2) || (n2 = et(t2)).type !== e2)
      throw st("Incompatible receiver, " + e2 + " required");
    return n2;
  };
} };
var vt = l(function(e2) {
  var t2 = Qe.CONFIGURABLE, n2 = yt.enforce, r2 = yt.get, o2 = Object.defineProperty, i2 = v && !y(function() {
    return 8 !== o2(function() {
    }, "length", { value: 8 }).length;
  }), c2 = String(String).split("String"), a2 = e2.exports = function(e3, r3, a3) {
    "Symbol(" === String(r3).slice(0, 7) && (r3 = "[" + String(r3).replace(/^Symbol\(([^)]*)\)/, "$1") + "]"), a3 && a3.getter && (r3 = "get " + r3), a3 && a3.setter && (r3 = "set " + r3), (!ge(e3, "name") || t2 && e3.name !== r3) && (v ? o2(e3, "name", { value: r3, configurable: true }) : e3.name = r3), i2 && a3 && ge(a3, "arity") && e3.length !== a3.arity && o2(e3, "length", { value: a3.arity });
    try {
      a3 && ge(a3, "constructor") && a3.constructor ? v && o2(e3, "prototype", { writable: false }) : e3.prototype && (e3.prototype = void 0);
    } catch (e4) {
    }
    var s2 = n2(e3);
    return ge(s2, "source") || (s2.source = c2.join("string" == typeof r3 ? r3 : "")), e3;
  };
  Function.prototype.toString = a2(function() {
    return H(this) && r2(this).source || nt(this);
  }, "toString");
});
var mt = function(e2, t2, n2, r2) {
  r2 || (r2 = {});
  var o2 = r2.enumerable, i2 = void 0 !== r2.name ? r2.name : t2;
  if (H(n2) && vt(n2, i2, r2), r2.global)
    o2 ? e2[t2] = n2 : he(t2, n2);
  else {
    try {
      r2.unsafe ? e2[t2] && (o2 = true) : delete e2[t2];
    } catch (e3) {
    }
    o2 ? e2[t2] = n2 : Je.f(e2, t2, { value: n2, enumerable: false, configurable: !r2.nonConfigurable, writable: !r2.nonWritable });
  }
  return e2;
};
var bt = Math.ceil;
var gt = Math.floor;
var wt = Math.trunc || function(e2) {
  var t2 = +e2;
  return (t2 > 0 ? gt : bt)(t2);
};
var St = function(e2) {
  var t2 = +e2;
  return t2 != t2 || 0 === t2 ? 0 : wt(t2);
};
var kt = Math.max;
var _t = Math.min;
var It = function(e2, t2) {
  var n2 = St(e2);
  return n2 < 0 ? kt(n2 + t2, 0) : _t(n2, t2);
};
var Ot = Math.min;
var xt = function(e2) {
  return e2 > 0 ? Ot(St(e2), 9007199254740991) : 0;
};
var Tt = function(e2) {
  return xt(e2.length);
};
var Ct = function(e2) {
  return function(t2, n2, r2) {
    var o2, i2 = K(t2), c2 = Tt(i2), a2 = It(r2, c2);
    if (e2 && n2 != n2) {
      for (; c2 > a2; )
        if ((o2 = i2[a2++]) != o2)
          return true;
    } else
      for (; c2 > a2; a2++)
        if ((e2 || a2 in i2) && i2[a2] === n2)
          return e2 || a2 || 0;
    return !e2 && -1;
  };
};
var jt = { includes: Ct(true), indexOf: Ct(false) };
var Lt = jt.indexOf;
var Rt = L([].push);
var Wt = function(e2, t2) {
  var n2, r2 = K(e2), o2 = 0, i2 = [];
  for (n2 in r2)
    !ge(at, n2) && ge(r2, n2) && Rt(i2, n2);
  for (; t2.length > o2; )
    ge(r2, n2 = t2[o2++]) && (~Lt(i2, n2) || Rt(i2, n2));
  return i2;
};
var Zt = ["constructor", "hasOwnProperty", "isPrototypeOf", "propertyIsEnumerable", "toLocaleString", "toString", "valueOf"];
var Et = Zt.concat("length", "prototype");
var Gt = { f: Object.getOwnPropertyNames || function(e2) {
  return Wt(e2, Et);
} };
var At = { f: Object.getOwnPropertySymbols };
var Pt = L([].concat);
var Xt = z("Reflect", "ownKeys") || function(e2) {
  var t2 = Gt.f(Ue(e2)), n2 = At.f;
  return n2 ? Pt(t2, n2(e2)) : t2;
};
var Ft = function(e2, t2, n2) {
  for (var r2 = Xt(t2), o2 = Je.f, i2 = Xe.f, c2 = 0; c2 < r2.length; c2++) {
    var a2 = r2[c2];
    ge(e2, a2) || n2 && ge(n2, a2) || o2(e2, a2, i2(t2, a2));
  }
};
var Kt = /#|\.prototype\./;
var Nt = function(e2, t2) {
  var n2 = Dt[Ut(e2)];
  return n2 == Yt || n2 != Ht && (H(t2) ? y(t2) : !!t2);
};
var Ut = Nt.normalize = function(e2) {
  return String(e2).replace(Kt, ".").toLowerCase();
};
var Dt = Nt.data = {};
var Ht = Nt.NATIVE = "N";
var Yt = Nt.POLYFILL = "P";
var Jt = Nt;
var Vt = Xe.f;
var zt = function(e2, t2) {
  var n2, r2, o2, i2, c2, a2 = e2.target, s2 = e2.global, u2 = e2.stat;
  if (n2 = s2 ? p : u2 ? p[a2] || he(a2, {}) : (p[a2] || {}).prototype)
    for (r2 in t2) {
      if (i2 = t2[r2], o2 = e2.dontCallGetSet ? (c2 = Vt(n2, r2)) && c2.value : n2[r2], !Jt(s2 ? r2 : a2 + (u2 ? "." : "#") + r2, e2.forced) && void 0 !== o2) {
        if (typeof i2 == typeof o2)
          continue;
        Ft(i2, o2);
      }
      (e2.sham || o2 && o2.sham) && Ve(i2, "sham", true), mt(n2, r2, i2, e2);
    }
};
var Mt = {};
Mt[Ce("toStringTag")] = "z";
var Bt;
var Qt = "[object z]" === String(Mt);
var qt = Ce("toStringTag");
var $t = Object;
var en = "Arguments" == Z(function() {
  return arguments;
}());
var tn = Qt ? Z : function(e2) {
  var t2, n2, r2;
  return void 0 === e2 ? "Undefined" : null === e2 ? "Null" : "string" == typeof (n2 = function(e3, t3) {
    try {
      return e3[t3];
    } catch (e4) {
    }
  }(t2 = $t(e2), qt)) ? n2 : en ? Z(t2) : "Object" == (r2 = Z(t2)) && H(t2.callee) ? "Arguments" : r2;
};
var nn = String;
var rn = function(e2) {
  if ("Symbol" === tn(e2))
    throw TypeError("Cannot convert a Symbol value to a string");
  return nn(e2);
};
var on = Ce("match");
var cn = TypeError;
var an = function(e2) {
  if (function(e3) {
    var t2;
    return J(e3) && (void 0 !== (t2 = e3[on]) ? !!t2 : "RegExp" == Z(e3));
  }(e2))
    throw cn("The method doesn't accept regular expressions");
  return e2;
};
var sn = Ce("match");
var un = function(e2) {
  var t2 = /./;
  try {
    "/./"[e2](t2);
  } catch (n2) {
    try {
      return t2[sn] = false, "/./"[e2](t2);
    } catch (e3) {
    }
  }
  return false;
};
var ln = Xe.f;
var fn = L("".startsWith);
var dn = L("".slice);
var hn = Math.min;
var pn = un("startsWith");
var yn = !(pn || (Bt = ln(String.prototype, "startsWith"), !Bt || Bt.writable));
zt({ target: "String", proto: true, forced: !yn && !pn }, { startsWith: function(e2) {
  var t2 = rn(F(this));
  an(e2);
  var n2 = xt(hn(arguments.length > 1 ? arguments[1] : void 0, t2.length)), r2 = rn(e2);
  return fn ? fn(t2, r2, n2) : dn(t2, n2, n2 + r2.length) === r2;
} });
var vn = function(e2, t2) {
  return L(p[e2].prototype[t2]);
};
vn("String", "startsWith");
var mn = Array.isArray || function(e2) {
  return "Array" == Z(e2);
};
var bn = TypeError;
var gn = function(e2) {
  if (e2 > 9007199254740991)
    throw bn("Maximum allowed index exceeded");
  return e2;
};
var wn = function(e2, t2, n2) {
  var r2 = We(t2);
  r2 in e2 ? Je.f(e2, r2, I(0, n2)) : e2[r2] = n2;
};
var Sn = function() {
};
var kn = [];
var _n = z("Reflect", "construct");
var In = /^\s*(?:class|function)\b/;
var On = L(In.exec);
var xn = !In.exec(Sn);
var Tn = function(e2) {
  if (!H(e2))
    return false;
  try {
    return _n(Sn, kn, e2), true;
  } catch (e3) {
    return false;
  }
};
var Cn = function(e2) {
  if (!H(e2))
    return false;
  switch (tn(e2)) {
    case "AsyncFunction":
    case "GeneratorFunction":
    case "AsyncGeneratorFunction":
      return false;
  }
  try {
    return xn || !!On(In, nt(e2));
  } catch (e3) {
    return true;
  }
};
Cn.sham = true;
var jn;
var Ln = !_n || y(function() {
  var e2;
  return Tn(Tn.call) || !Tn(Object) || !Tn(function() {
    e2 = true;
  }) || e2;
}) ? Cn : Tn;
var Rn = Ce("species");
var Wn = Array;
var Zn = function(e2, t2) {
  return new (function(e3) {
    var t3;
    return mn(e3) && (t3 = e3.constructor, (Ln(t3) && (t3 === Wn || mn(t3.prototype)) || J(t3) && null === (t3 = t3[Rn])) && (t3 = void 0)), void 0 === t3 ? Wn : t3;
  }(e2))(0 === t2 ? 0 : t2);
};
var En = Ce("species");
var Gn = Ce("isConcatSpreadable");
var An = te >= 51 || !y(function() {
  var e2 = [];
  return e2[Gn] = false, e2.concat()[0] !== e2;
});
var Pn = (jn = "concat", te >= 51 || !y(function() {
  var e2 = [];
  return (e2.constructor = {})[En] = function() {
    return { foo: 1 };
  }, 1 !== e2[jn](Boolean).foo;
}));
var Xn = function(e2) {
  if (!J(e2))
    return false;
  var t2 = e2[Gn];
  return void 0 !== t2 ? !!t2 : mn(e2);
};
zt({ target: "Array", proto: true, arity: 1, forced: !An || !Pn }, { concat: function(e2) {
  var t2, n2, r2, o2, i2, c2 = me(this), a2 = Zn(c2, 0), s2 = 0;
  for (t2 = -1, r2 = arguments.length; t2 < r2; t2++)
    if (Xn(i2 = -1 === t2 ? c2 : arguments[t2]))
      for (o2 = Tt(i2), gn(s2 + o2), n2 = 0; n2 < o2; n2++, s2++)
        n2 in i2 && wn(a2, s2, i2[n2]);
    else
      gn(s2 + 1), wn(a2, s2++, i2);
  return a2.length = s2, a2;
} });
var Fn = Qt ? {}.toString : function() {
  return "[object " + tn(this) + "]";
};
Qt || mt(Object.prototype, "toString", Fn, { unsafe: true });
var Kn;
var Nn = Object.keys || function(e2) {
  return Wt(e2, Zt);
};
var Un = v && !Fe ? Object.defineProperties : function(e2, t2) {
  Ue(e2);
  for (var n2, r2 = K(t2), o2 = Nn(t2), i2 = o2.length, c2 = 0; i2 > c2; )
    Je.f(e2, n2 = o2[c2++], r2[n2]);
  return e2;
};
var Dn = { f: Un };
var Hn = z("document", "documentElement");
var Yn = ct("IE_PROTO");
var Jn = function() {
};
var Vn = function(e2) {
  return "<script>" + e2 + "<\/script>";
};
var zn = function(e2) {
  e2.write(Vn("")), e2.close();
  var t2 = e2.parentWindow.Object;
  return e2 = null, t2;
};
var Mn = function() {
  try {
    Kn = new ActiveXObject("htmlfile");
  } catch (e3) {
  }
  var e2, t2;
  Mn = "undefined" != typeof document ? document.domain && Kn ? zn(Kn) : ((t2 = Ge("iframe")).style.display = "none", Hn.appendChild(t2), t2.src = String("javascript:"), (e2 = t2.contentWindow.document).open(), e2.write(Vn("document.F=Object")), e2.close(), e2.F) : zn(Kn);
  for (var n2 = Zt.length; n2--; )
    delete Mn.prototype[Zt[n2]];
  return Mn();
};
at[Yn] = true;
var Bn = Object.create || function(e2, t2) {
  var n2;
  return null !== e2 ? (Jn.prototype = Ue(e2), n2 = new Jn(), Jn.prototype = null, n2[Yn] = e2) : n2 = Mn(), void 0 === t2 ? n2 : Dn.f(n2, t2);
};
var Qn = Array;
var qn = Math.max;
var $n = Gt.f;
var er = "object" == typeof window && window && Object.getOwnPropertyNames ? Object.getOwnPropertyNames(window) : [];
var tr = function(e2) {
  try {
    return $n(e2);
  } catch (e3) {
    return function(e4, t2, n2) {
      for (var r2 = Tt(e4), o2 = It(t2, r2), i2 = It(void 0 === n2 ? r2 : n2, r2), c2 = Qn(qn(i2 - o2, 0)), a2 = 0; o2 < i2; o2++, a2++)
        wn(c2, a2, e4[o2]);
      return c2.length = a2, c2;
    }(er);
  }
};
var nr = { f: function(e2) {
  return er && "Window" == Z(e2) ? tr(e2) : $n(K(e2));
} };
var rr = { f: Ce };
var or = p;
var ir = Je.f;
var cr = function(e2) {
  var t2 = or.Symbol || (or.Symbol = {});
  ge(t2, e2) || ir(t2, e2, { value: rr.f(e2) });
};
var ar = function() {
  var e2 = z("Symbol"), t2 = e2 && e2.prototype, n2 = t2 && t2.valueOf, r2 = Ce("toPrimitive");
  t2 && !t2[r2] && mt(t2, r2, function(e3) {
    return g(n2, this);
  }, { arity: 1 });
};
var sr = Je.f;
var ur = Ce("toStringTag");
var lr = function(e2, t2, n2) {
  e2 && !n2 && (e2 = e2.prototype), e2 && !ge(e2, ur) && sr(e2, ur, { configurable: true, value: t2 });
};
var fr = L(L.bind);
var dr = function(e2, t2) {
  return ue(e2), void 0 === t2 ? e2 : m ? fr(e2, t2) : function() {
    return e2.apply(t2, arguments);
  };
};
var hr = L([].push);
var pr = function(e2) {
  var t2 = 1 == e2, n2 = 2 == e2, r2 = 3 == e2, o2 = 4 == e2, i2 = 6 == e2, c2 = 7 == e2, a2 = 5 == e2 || i2;
  return function(s2, u2, l2, f2) {
    for (var d2, h2, p2 = me(s2), y2 = A(p2), v2 = dr(u2, l2), m2 = Tt(y2), b2 = 0, g2 = f2 || Zn, w2 = t2 ? g2(s2, m2) : n2 || c2 ? g2(s2, 0) : void 0; m2 > b2; b2++)
      if ((a2 || b2 in y2) && (h2 = v2(d2 = y2[b2], b2, p2), e2))
        if (t2)
          w2[b2] = h2;
        else if (h2)
          switch (e2) {
            case 3:
              return true;
            case 5:
              return d2;
            case 6:
              return b2;
            case 2:
              hr(w2, d2);
          }
        else
          switch (e2) {
            case 4:
              return false;
            case 7:
              hr(w2, d2);
          }
    return i2 ? -1 : r2 || o2 ? o2 : w2;
  };
};
var yr = { forEach: pr(0), map: pr(1), filter: pr(2), some: pr(3), every: pr(4), find: pr(5), findIndex: pr(6), filterReject: pr(7) }.forEach;
var vr = ct("hidden");
var mr = yt.set;
var br = yt.getterFor("Symbol");
var gr = Object.prototype;
var wr = p.Symbol;
var Sr = wr && wr.prototype;
var kr = p.TypeError;
var _r = p.QObject;
var Ir = Xe.f;
var Or = Je.f;
var xr = nr.f;
var Tr = _.f;
var Cr = L([].push);
var jr = ye("symbols");
var Lr = ye("op-symbols");
var Rr = ye("wks");
var Wr = !_r || !_r.prototype || !_r.prototype.findChild;
var Zr = v && y(function() {
  return 7 != Bn(Or({}, "a", { get: function() {
    return Or(this, "a", { value: 7 }).a;
  } })).a;
}) ? function(e2, t2, n2) {
  var r2 = Ir(gr, t2);
  r2 && delete gr[t2], Or(e2, t2, n2), r2 && e2 !== gr && Or(gr, t2, r2);
} : Or;
var Er = function(e2, t2) {
  var n2 = jr[e2] = Bn(Sr);
  return mr(n2, { type: "Symbol", tag: e2, description: t2 }), v || (n2.description = t2), n2;
};
var Gr = function(e2, t2, n2) {
  e2 === gr && Gr(Lr, t2, n2), Ue(e2);
  var r2 = We(t2);
  return Ue(n2), ge(jr, r2) ? (n2.enumerable ? (ge(e2, vr) && e2[vr][r2] && (e2[vr][r2] = false), n2 = Bn(n2, { enumerable: I(0, false) })) : (ge(e2, vr) || Or(e2, vr, I(1, {})), e2[vr][r2] = true), Zr(e2, r2, n2)) : Or(e2, r2, n2);
};
var Ar = function(e2, t2) {
  Ue(e2);
  var n2 = K(t2), r2 = Nn(n2).concat(Kr(n2));
  return yr(r2, function(t3) {
    v && !g(Pr, n2, t3) || Gr(e2, t3, n2[t3]);
  }), e2;
};
var Pr = function(e2) {
  var t2 = We(e2), n2 = g(Tr, this, t2);
  return !(this === gr && ge(jr, t2) && !ge(Lr, t2)) && (!(n2 || !ge(this, t2) || !ge(jr, t2) || ge(this, vr) && this[vr][t2]) || n2);
};
var Xr = function(e2, t2) {
  var n2 = K(e2), r2 = We(t2);
  if (n2 !== gr || !ge(jr, r2) || ge(Lr, r2)) {
    var o2 = Ir(n2, r2);
    return !o2 || !ge(jr, r2) || ge(n2, vr) && n2[vr][r2] || (o2.enumerable = true), o2;
  }
};
var Fr = function(e2) {
  var t2 = xr(K(e2)), n2 = [];
  return yr(t2, function(e3) {
    ge(jr, e3) || ge(at, e3) || Cr(n2, e3);
  }), n2;
};
var Kr = function(e2) {
  var t2 = e2 === gr, n2 = xr(t2 ? Lr : K(e2)), r2 = [];
  return yr(n2, function(e3) {
    !ge(jr, e3) || t2 && !ge(gr, e3) || Cr(r2, jr[e3]);
  }), r2;
};
ne || (Sr = (wr = function() {
  if (M(Sr, this))
    throw kr("Symbol is not a constructor");
  var e2 = arguments.length && void 0 !== arguments[0] ? rn(arguments[0]) : void 0, t2 = _e(e2), n2 = function(e3) {
    this === gr && g(n2, Lr, e3), ge(this, vr) && ge(this[vr], t2) && (this[vr][t2] = false), Zr(this, t2, I(1, e3));
  };
  return v && Wr && Zr(gr, t2, { configurable: true, set: n2 }), Er(t2, e2);
}).prototype, mt(Sr, "toString", function() {
  return br(this).tag;
}), mt(wr, "withoutSetter", function(e2) {
  return Er(_e(e2), e2);
}), _.f = Pr, Je.f = Gr, Dn.f = Ar, Xe.f = Xr, Gt.f = nr.f = Fr, At.f = Kr, rr.f = function(e2) {
  return Er(Ce(e2), e2);
}, v && (Or(Sr, "description", { configurable: true, get: function() {
  return br(this).description;
} }), mt(gr, "propertyIsEnumerable", Pr, { unsafe: true }))), zt({ global: true, constructor: true, wrap: true, forced: !ne, sham: !ne }, { Symbol: wr }), yr(Nn(Rr), function(e2) {
  cr(e2);
}), zt({ target: "Symbol", stat: true, forced: !ne }, { useSetter: function() {
  Wr = true;
}, useSimple: function() {
  Wr = false;
} }), zt({ target: "Object", stat: true, forced: !ne, sham: !v }, { create: function(e2, t2) {
  return void 0 === t2 ? Bn(e2) : Ar(Bn(e2), t2);
}, defineProperty: Gr, defineProperties: Ar, getOwnPropertyDescriptor: Xr }), zt({ target: "Object", stat: true, forced: !ne }, { getOwnPropertyNames: Fr }), ar(), lr(wr, "Symbol"), at[vr] = true;
var Nr = ne && !!Symbol.for && !!Symbol.keyFor;
var Ur = ye("string-to-symbol-registry");
var Dr = ye("symbol-to-string-registry");
zt({ target: "Symbol", stat: true, forced: !Nr }, { for: function(e2) {
  var t2 = rn(e2);
  if (ge(Ur, t2))
    return Ur[t2];
  var n2 = z("Symbol")(t2);
  return Ur[t2] = n2, Dr[n2] = t2, n2;
} });
var Hr = ye("symbol-to-string-registry");
zt({ target: "Symbol", stat: true, forced: !Nr }, { keyFor: function(e2) {
  if (!ie(e2))
    throw TypeError(ae(e2) + " is not a symbol");
  if (ge(Hr, e2))
    return Hr[e2];
} });
var Yr = Function.prototype;
var Jr = Yr.apply;
var Vr = Yr.call;
var zr = "object" == typeof Reflect && Reflect.apply || (m ? Vr.bind(Jr) : function() {
  return Vr.apply(Jr, arguments);
});
var Mr = L([].slice);
var Br = z("JSON", "stringify");
var Qr = L(/./.exec);
var qr = L("".charAt);
var $r = L("".charCodeAt);
var eo = L("".replace);
var to = L(1 .toString);
var no = /[\uD800-\uDFFF]/g;
var ro = /^[\uD800-\uDBFF]$/;
var oo = /^[\uDC00-\uDFFF]$/;
var io = !ne || y(function() {
  var e2 = z("Symbol")();
  return "[null]" != Br([e2]) || "{}" != Br({ a: e2 }) || "{}" != Br(Object(e2));
});
var co = y(function() {
  return '"\\udf06\\ud834"' !== Br("\uDF06\uD834") || '"\\udead"' !== Br("\uDEAD");
});
var ao = function(e2, t2) {
  var n2 = Mr(arguments), r2 = t2;
  if ((J(t2) || void 0 !== e2) && !ie(e2))
    return mn(t2) || (t2 = function(e3, t3) {
      if (H(r2) && (t3 = g(r2, this, e3, t3)), !ie(t3))
        return t3;
    }), n2[1] = t2, zr(Br, null, n2);
};
var so = function(e2, t2, n2) {
  var r2 = qr(n2, t2 - 1), o2 = qr(n2, t2 + 1);
  return Qr(ro, e2) && !Qr(oo, o2) || Qr(oo, e2) && !Qr(ro, r2) ? "\\u" + to($r(e2, 0), 16) : e2;
};
Br && zt({ target: "JSON", stat: true, arity: 3, forced: io || co }, { stringify: function(e2, t2, n2) {
  var r2 = Mr(arguments), o2 = zr(io ? ao : Br, null, r2);
  return co && "string" == typeof o2 ? eo(o2, no, so) : o2;
} });
var uo = !ne || y(function() {
  At.f(1);
});
zt({ target: "Object", stat: true, forced: uo }, { getOwnPropertySymbols: function(e2) {
  var t2 = At.f;
  return t2 ? t2(me(e2)) : [];
} }), cr("asyncIterator");
var lo = Je.f;
var fo = p.Symbol;
var ho = fo && fo.prototype;
if (v && H(fo) && (!("description" in ho) || void 0 !== fo().description)) {
  po = {}, yo = function() {
    var e2 = arguments.length < 1 || void 0 === arguments[0] ? void 0 : rn(arguments[0]), t2 = M(ho, this) ? new fo(e2) : void 0 === e2 ? fo() : fo(e2);
    return "" === e2 && (po[t2] = true), t2;
  };
  Ft(yo, fo), yo.prototype = ho, ho.constructor = yo;
  vo = "Symbol(test)" == String(fo("test")), mo = L(ho.valueOf), bo = L(ho.toString), go = /^Symbol\((.*)\)[^)]+$/, wo = L("".replace), So = L("".slice);
  lo(ho, "description", { configurable: true, get: function() {
    var e2 = mo(this);
    if (ge(po, e2))
      return "";
    var t2 = bo(e2), n2 = vo ? So(t2, 7, -1) : wo(t2, go, "$1");
    return "" === n2 ? void 0 : n2;
  } }), zt({ global: true, constructor: true, forced: true }, { Symbol: yo });
}
var po;
var yo;
var vo;
var mo;
var bo;
var go;
var wo;
var So;
cr("hasInstance"), cr("isConcatSpreadable"), cr("iterator"), cr("match"), cr("matchAll"), cr("replace"), cr("search"), cr("species"), cr("split"), cr("toPrimitive"), ar(), cr("toStringTag"), lr(z("Symbol"), "Symbol"), cr("unscopables"), lr(p.JSON, "JSON", true), lr(Math, "Math", true), zt({ global: true }, { Reflect: {} }), lr(p.Reflect, "Reflect", true), or.Symbol;
var ko;
var _o;
var Io;
var Oo = L("".charAt);
var xo = L("".charCodeAt);
var To = L("".slice);
var Co = function(e2) {
  return function(t2, n2) {
    var r2, o2, i2 = rn(F(t2)), c2 = St(n2), a2 = i2.length;
    return c2 < 0 || c2 >= a2 ? e2 ? "" : void 0 : (r2 = xo(i2, c2)) < 55296 || r2 > 56319 || c2 + 1 === a2 || (o2 = xo(i2, c2 + 1)) < 56320 || o2 > 57343 ? e2 ? Oo(i2, c2) : r2 : e2 ? To(i2, c2, c2 + 2) : o2 - 56320 + (r2 - 55296 << 10) + 65536;
  };
};
var jo = { codeAt: Co(false), charAt: Co(true) };
var Lo = !y(function() {
  function e2() {
  }
  return e2.prototype.constructor = null, Object.getPrototypeOf(new e2()) !== e2.prototype;
});
var Ro = ct("IE_PROTO");
var Wo = Object;
var Zo = Wo.prototype;
var Eo = Lo ? Wo.getPrototypeOf : function(e2) {
  var t2 = me(e2);
  if (ge(t2, Ro))
    return t2[Ro];
  var n2 = t2.constructor;
  return H(n2) && t2 instanceof n2 ? n2.prototype : t2 instanceof Wo ? Zo : null;
};
var Go = Ce("iterator");
var Ao = false;
[].keys && ("next" in (Io = [].keys()) ? (_o = Eo(Eo(Io))) !== Object.prototype && (ko = _o) : Ao = true);
var Po = !J(ko) || y(function() {
  var e2 = {};
  return ko[Go].call(e2) !== e2;
});
Po && (ko = {}), H(ko[Go]) || mt(ko, Go, function() {
  return this;
});
var Xo = { IteratorPrototype: ko, BUGGY_SAFARI_ITERATORS: Ao };
var Fo = {};
var Ko = Xo.IteratorPrototype;
var No = function() {
  return this;
};
var Uo = String;
var Do = TypeError;
var Ho = Object.setPrototypeOf || ("__proto__" in {} ? function() {
  var e2, t2 = false, n2 = {};
  try {
    (e2 = L(Object.getOwnPropertyDescriptor(Object.prototype, "__proto__").set))(n2, []), t2 = n2 instanceof Array;
  } catch (e3) {
  }
  return function(n3, r2) {
    return Ue(n3), function(e3) {
      if ("object" == typeof e3 || H(e3))
        return e3;
      throw Do("Can't set " + Uo(e3) + " as a prototype");
    }(r2), t2 ? e2(n3, r2) : n3.__proto__ = r2, n3;
  };
}() : void 0);
var Yo = Qe.PROPER;
var Jo = Qe.CONFIGURABLE;
var Vo = Xo.IteratorPrototype;
var zo = Xo.BUGGY_SAFARI_ITERATORS;
var Mo = Ce("iterator");
var Bo = function() {
  return this;
};
var Qo = function(e2, t2, n2, r2, o2, i2, c2) {
  !function(e3, t3, n3, r3) {
    var o3 = t3 + " Iterator";
    e3.prototype = Bn(Ko, { next: I(+!r3, n3) }), lr(e3, o3, false), Fo[o3] = No;
  }(n2, t2, r2);
  var a2, s2, u2, l2 = function(e3) {
    if (e3 === o2 && y2)
      return y2;
    if (!zo && e3 in h2)
      return h2[e3];
    switch (e3) {
      case "keys":
      case "values":
      case "entries":
        return function() {
          return new n2(this, e3);
        };
    }
    return function() {
      return new n2(this);
    };
  }, f2 = t2 + " Iterator", d2 = false, h2 = e2.prototype, p2 = h2[Mo] || h2["@@iterator"] || o2 && h2[o2], y2 = !zo && p2 || l2(o2), v2 = "Array" == t2 && h2.entries || p2;
  if (v2 && (a2 = Eo(v2.call(new e2()))) !== Object.prototype && a2.next && (Eo(a2) !== Vo && (Ho ? Ho(a2, Vo) : H(a2[Mo]) || mt(a2, Mo, Bo)), lr(a2, f2, true)), Yo && "values" == o2 && p2 && "values" !== p2.name && (Jo ? Ve(h2, "name", "values") : (d2 = true, y2 = function() {
    return g(p2, this);
  })), o2)
    if (s2 = { values: l2("values"), keys: i2 ? y2 : l2("keys"), entries: l2("entries") }, c2)
      for (u2 in s2)
        (zo || d2 || !(u2 in h2)) && mt(h2, u2, s2[u2]);
    else
      zt({ target: t2, proto: true, forced: zo || d2 }, s2);
  return h2[Mo] !== y2 && mt(h2, Mo, y2, { name: o2 }), Fo[t2] = y2, s2;
};
var qo = function(e2, t2) {
  return { value: e2, done: t2 };
};
var $o = jo.charAt;
var ei = yt.set;
var ti = yt.getterFor("String Iterator");
Qo(String, "String", function(e2) {
  ei(this, { type: "String Iterator", string: rn(e2), index: 0 });
}, function() {
  var e2, t2 = ti(this), n2 = t2.string, r2 = t2.index;
  return r2 >= n2.length ? qo(void 0, true) : (e2 = $o(n2, r2), t2.index += e2.length, qo(e2, false));
});
var ni = function(e2, t2, n2) {
  var r2, o2;
  Ue(e2);
  try {
    if (!(r2 = le(e2, "return"))) {
      if ("throw" === t2)
        throw n2;
      return n2;
    }
    r2 = g(r2, e2);
  } catch (e3) {
    o2 = true, r2 = e3;
  }
  if ("throw" === t2)
    throw n2;
  if (o2)
    throw r2;
  return Ue(r2), n2;
};
var ri = function(e2, t2, n2, r2) {
  try {
    return r2 ? t2(Ue(n2)[0], n2[1]) : t2(n2);
  } catch (t3) {
    ni(e2, "throw", t3);
  }
};
var oi = Ce("iterator");
var ii = Array.prototype;
var ci = function(e2) {
  return void 0 !== e2 && (Fo.Array === e2 || ii[oi] === e2);
};
var ai = Ce("iterator");
var si = function(e2) {
  if (!P(e2))
    return le(e2, ai) || le(e2, "@@iterator") || Fo[tn(e2)];
};
var ui = TypeError;
var li = function(e2, t2) {
  var n2 = arguments.length < 2 ? si(e2) : t2;
  if (ue(n2))
    return Ue(g(n2, e2));
  throw ui(ae(e2) + " is not iterable");
};
var fi = Array;
var di = Ce("iterator");
var hi = false;
try {
  pi = 0, yi = { next: function() {
    return { done: !!pi++ };
  }, return: function() {
    hi = true;
  } };
  yi[di] = function() {
    return this;
  }, Array.from(yi, function() {
    throw 2;
  });
} catch (e2) {
}
var pi;
var yi;
var vi = function(e2, t2) {
  if (!t2 && !hi)
    return false;
  var n2 = false;
  try {
    var r2 = {};
    r2[di] = function() {
      return { next: function() {
        return { done: n2 = true };
      } };
    }, e2(r2);
  } catch (e3) {
  }
  return n2;
};
var mi = !vi(function(e2) {
  Array.from(e2);
});
zt({ target: "Array", stat: true, forced: mi }, { from: function(e2) {
  var t2 = me(e2), n2 = Ln(this), r2 = arguments.length, o2 = r2 > 1 ? arguments[1] : void 0, i2 = void 0 !== o2;
  i2 && (o2 = dr(o2, r2 > 2 ? arguments[2] : void 0));
  var c2, a2, s2, u2, l2, f2, d2 = si(t2), h2 = 0;
  if (!d2 || this === fi && ci(d2))
    for (c2 = Tt(t2), a2 = n2 ? new this(c2) : fi(c2); c2 > h2; h2++)
      f2 = i2 ? o2(t2[h2], h2) : t2[h2], wn(a2, h2, f2);
  else
    for (l2 = (u2 = li(t2, d2)).next, a2 = n2 ? new this() : []; !(s2 = g(l2, u2)).done; h2++)
      f2 = i2 ? ri(u2, o2, [s2.value, h2], true) : s2.value, wn(a2, h2, f2);
  return a2.length = h2, a2;
} }), or.Array.from;
var bi;
var gi;
var wi;
var Si = "undefined" != typeof ArrayBuffer && "undefined" != typeof DataView;
var ki = Je.f;
var _i = yt.enforce;
var Ii = yt.get;
var Oi = p.Int8Array;
var xi = Oi && Oi.prototype;
var Ti = p.Uint8ClampedArray;
var Ci = Ti && Ti.prototype;
var ji = Oi && Eo(Oi);
var Li = xi && Eo(xi);
var Ri = Object.prototype;
var Wi = p.TypeError;
var Zi = Ce("toStringTag");
var Ei = _e("TYPED_ARRAY_TAG");
var Gi = Si && !!Ho && "Opera" !== tn(p.opera);
var Ai = false;
var Pi = { Int8Array: 1, Uint8Array: 1, Uint8ClampedArray: 1, Int16Array: 2, Uint16Array: 2, Int32Array: 4, Uint32Array: 4, Float32Array: 4, Float64Array: 8 };
var Xi = { BigInt64Array: 8, BigUint64Array: 8 };
var Fi = function(e2) {
  var t2 = Eo(e2);
  if (J(t2)) {
    var n2 = Ii(t2);
    return n2 && ge(n2, "TypedArrayConstructor") ? n2.TypedArrayConstructor : Fi(t2);
  }
};
var Ki = function(e2) {
  if (!J(e2))
    return false;
  var t2 = tn(e2);
  return ge(Pi, t2) || ge(Xi, t2);
};
for (bi in Pi)
  (wi = (gi = p[bi]) && gi.prototype) ? _i(wi).TypedArrayConstructor = gi : Gi = false;
for (bi in Xi)
  (wi = (gi = p[bi]) && gi.prototype) && (_i(wi).TypedArrayConstructor = gi);
if ((!Gi || !H(ji) || ji === Function.prototype) && (ji = function() {
  throw Wi("Incorrect invocation");
}, Gi))
  for (bi in Pi)
    p[bi] && Ho(p[bi], ji);
if ((!Gi || !Li || Li === Ri) && (Li = ji.prototype, Gi))
  for (bi in Pi)
    p[bi] && Ho(p[bi].prototype, Li);
if (Gi && Eo(Ci) !== Li && Ho(Ci, Li), v && !ge(Li, Zi))
  for (bi in Ai = true, ki(Li, Zi, { get: function() {
    return J(this) ? this[Ei] : void 0;
  } }), Pi)
    p[bi] && Ve(p[bi], Ei, bi);
var Ni = { NATIVE_ARRAY_BUFFER_VIEWS: Gi, TYPED_ARRAY_TAG: Ai && Ei, aTypedArray: function(e2) {
  if (Ki(e2))
    return e2;
  throw Wi("Target is not a typed array");
}, aTypedArrayConstructor: function(e2) {
  if (H(e2) && (!Ho || M(ji, e2)))
    return e2;
  throw Wi(ae(e2) + " is not a typed array constructor");
}, exportTypedArrayMethod: function(e2, t2, n2, r2) {
  if (v) {
    if (n2)
      for (var o2 in Pi) {
        var i2 = p[o2];
        if (i2 && ge(i2.prototype, e2))
          try {
            delete i2.prototype[e2];
          } catch (n3) {
            try {
              i2.prototype[e2] = t2;
            } catch (e3) {
            }
          }
      }
    Li[e2] && !n2 || mt(Li, e2, n2 ? t2 : Gi && xi[e2] || t2, r2);
  }
}, exportTypedArrayStaticMethod: function(e2, t2, n2) {
  var r2, o2;
  if (v) {
    if (Ho) {
      if (n2) {
        for (r2 in Pi)
          if ((o2 = p[r2]) && ge(o2, e2))
            try {
              delete o2[e2];
            } catch (e3) {
            }
      }
      if (ji[e2] && !n2)
        return;
      try {
        return mt(ji, e2, n2 ? t2 : Gi && ji[e2] || t2);
      } catch (e3) {
      }
    }
    for (r2 in Pi)
      !(o2 = p[r2]) || o2[e2] && !n2 || mt(o2, e2, t2);
  }
}, getTypedArrayConstructor: Fi, isView: function(e2) {
  if (!J(e2))
    return false;
  var t2 = tn(e2);
  return "DataView" === t2 || ge(Pi, t2) || ge(Xi, t2);
}, isTypedArray: Ki, TypedArray: ji, TypedArrayPrototype: Li };
var Ui = TypeError;
var Di = Ce("species");
var Hi = function(e2, t2) {
  var n2, r2 = Ue(e2).constructor;
  return void 0 === r2 || P(n2 = Ue(r2)[Di]) ? t2 : function(e3) {
    if (Ln(e3))
      return e3;
    throw Ui(ae(e3) + " is not a constructor");
  }(n2);
};
var Yi = Ni.aTypedArrayConstructor;
var Ji = Ni.getTypedArrayConstructor;
var Vi = Ni.aTypedArray;
(0, Ni.exportTypedArrayMethod)("slice", function(e2, t2) {
  for (var n2, r2 = Mr(Vi(this), e2, t2), o2 = Yi(Hi(n2 = this, Ji(n2))), i2 = 0, c2 = r2.length, a2 = new o2(c2); c2 > i2; )
    a2[i2] = r2[i2++];
  return a2;
}, y(function() {
  new Int8Array(1).slice();
}));
var zi = Je.f;
var Mi = Ce("unscopables");
var Bi = Array.prototype;
null == Bi[Mi] && zi(Bi, Mi, { configurable: true, value: Bn(null) });
var Qi = function(e2) {
  Bi[Mi][e2] = true;
};
var qi = jt.includes;
var $i = y(function() {
  return !Array(1).includes();
});
zt({ target: "Array", proto: true, forced: $i }, { includes: function(e2) {
  return qi(this, e2, arguments.length > 1 ? arguments[1] : void 0);
} }), Qi("includes"), vn("Array", "includes");
var ec = L("".indexOf);
zt({ target: "String", proto: true, forced: !un("includes") }, { includes: function(e2) {
  return !!~ec(rn(F(this)), rn(an(e2)), arguments.length > 1 ? arguments[1] : void 0);
} }), vn("String", "includes");
var tc = Je.f;
var nc = yt.set;
var rc = yt.getterFor("Array Iterator");
Qo(Array, "Array", function(e2, t2) {
  nc(this, { type: "Array Iterator", target: K(e2), index: 0, kind: t2 });
}, function() {
  var e2 = rc(this), t2 = e2.target, n2 = e2.kind, r2 = e2.index++;
  return !t2 || r2 >= t2.length ? (e2.target = void 0, qo(void 0, true)) : qo("keys" == n2 ? r2 : "values" == n2 ? t2[r2] : [r2, t2[r2]], false);
}, "values");
var oc = Fo.Arguments = Fo.Array;
if (Qi("keys"), Qi("values"), Qi("entries"), v && "values" !== oc.name)
  try {
    tc(oc, "name", { value: "values" });
  } catch (e2) {
  }
var ic = y(function() {
  if ("function" == typeof ArrayBuffer) {
    var e2 = new ArrayBuffer(8);
    Object.isExtensible(e2) && Object.defineProperty(e2, "a", { value: 8 });
  }
});
var cc = Object.isExtensible;
var ac = y(function() {
  cc(1);
}) || ic ? function(e2) {
  return !!J(e2) && ((!ic || "ArrayBuffer" != Z(e2)) && (!cc || cc(e2)));
} : cc;
var sc = !y(function() {
  return Object.isExtensible(Object.preventExtensions({}));
});
var uc = l(function(e2) {
  var t2 = Je.f, n2 = false, r2 = _e("meta"), o2 = 0, i2 = function(e3) {
    t2(e3, r2, { value: { objectID: "O" + o2++, weakData: {} } });
  }, c2 = e2.exports = { enable: function() {
    c2.enable = function() {
    }, n2 = true;
    var e3 = Gt.f, t3 = L([].splice), o3 = {};
    o3[r2] = 1, e3(o3).length && (Gt.f = function(n3) {
      for (var o4 = e3(n3), i3 = 0, c3 = o4.length; i3 < c3; i3++)
        if (o4[i3] === r2) {
          t3(o4, i3, 1);
          break;
        }
      return o4;
    }, zt({ target: "Object", stat: true, forced: true }, { getOwnPropertyNames: nr.f }));
  }, fastKey: function(e3, t3) {
    if (!J(e3))
      return "symbol" == typeof e3 ? e3 : ("string" == typeof e3 ? "S" : "P") + e3;
    if (!ge(e3, r2)) {
      if (!ac(e3))
        return "F";
      if (!t3)
        return "E";
      i2(e3);
    }
    return e3[r2].objectID;
  }, getWeakData: function(e3, t3) {
    if (!ge(e3, r2)) {
      if (!ac(e3))
        return true;
      if (!t3)
        return false;
      i2(e3);
    }
    return e3[r2].weakData;
  }, onFreeze: function(e3) {
    return sc && n2 && ac(e3) && !ge(e3, r2) && i2(e3), e3;
  } };
  at[r2] = true;
});
uc.enable, uc.fastKey, uc.getWeakData, uc.onFreeze;
var lc = TypeError;
var fc = function(e2, t2) {
  this.stopped = e2, this.result = t2;
};
var dc = fc.prototype;
var hc = function(e2, t2, n2) {
  var r2, o2, i2, c2, a2, s2, u2, l2 = n2 && n2.that, f2 = !(!n2 || !n2.AS_ENTRIES), d2 = !(!n2 || !n2.IS_RECORD), h2 = !(!n2 || !n2.IS_ITERATOR), p2 = !(!n2 || !n2.INTERRUPTED), y2 = dr(t2, l2), v2 = function(e3) {
    return r2 && ni(r2, "normal", e3), new fc(true, e3);
  }, m2 = function(e3) {
    return f2 ? (Ue(e3), p2 ? y2(e3[0], e3[1], v2) : y2(e3[0], e3[1])) : p2 ? y2(e3, v2) : y2(e3);
  };
  if (d2)
    r2 = e2.iterator;
  else if (h2)
    r2 = e2;
  else {
    if (!(o2 = si(e2)))
      throw lc(ae(e2) + " is not iterable");
    if (ci(o2)) {
      for (i2 = 0, c2 = Tt(e2); c2 > i2; i2++)
        if ((a2 = m2(e2[i2])) && M(dc, a2))
          return a2;
      return new fc(false);
    }
    r2 = li(e2, o2);
  }
  for (s2 = d2 ? e2.next : r2.next; !(u2 = g(s2, r2)).done; ) {
    try {
      a2 = m2(u2.value);
    } catch (e3) {
      ni(r2, "throw", e3);
    }
    if ("object" == typeof a2 && a2 && M(dc, a2))
      return a2;
  }
  return new fc(false);
};
var pc = TypeError;
var yc = function(e2, t2) {
  if (M(t2, e2))
    return e2;
  throw pc("Incorrect invocation");
};
var vc = function(e2, t2, n2) {
  for (var r2 in t2)
    mt(e2, r2, t2[r2], n2);
  return e2;
};
var mc = Ce("species");
var bc = Je.f;
var gc = uc.fastKey;
var wc = yt.set;
var Sc = yt.getterFor;
var kc = { getConstructor: function(e2, t2, n2, r2) {
  var o2 = e2(function(e3, o3) {
    yc(e3, i2), wc(e3, { type: t2, index: Bn(null), first: void 0, last: void 0, size: 0 }), v || (e3.size = 0), P(o3) || hc(o3, e3[r2], { that: e3, AS_ENTRIES: n2 });
  }), i2 = o2.prototype, c2 = Sc(t2), a2 = function(e3, t3, n3) {
    var r3, o3, i3 = c2(e3), a3 = s2(e3, t3);
    return a3 ? a3.value = n3 : (i3.last = a3 = { index: o3 = gc(t3, true), key: t3, value: n3, previous: r3 = i3.last, next: void 0, removed: false }, i3.first || (i3.first = a3), r3 && (r3.next = a3), v ? i3.size++ : e3.size++, "F" !== o3 && (i3.index[o3] = a3)), e3;
  }, s2 = function(e3, t3) {
    var n3, r3 = c2(e3), o3 = gc(t3);
    if ("F" !== o3)
      return r3.index[o3];
    for (n3 = r3.first; n3; n3 = n3.next)
      if (n3.key == t3)
        return n3;
  };
  return vc(i2, { clear: function() {
    for (var e3 = c2(this), t3 = e3.index, n3 = e3.first; n3; )
      n3.removed = true, n3.previous && (n3.previous = n3.previous.next = void 0), delete t3[n3.index], n3 = n3.next;
    e3.first = e3.last = void 0, v ? e3.size = 0 : this.size = 0;
  }, delete: function(e3) {
    var t3 = this, n3 = c2(t3), r3 = s2(t3, e3);
    if (r3) {
      var o3 = r3.next, i3 = r3.previous;
      delete n3.index[r3.index], r3.removed = true, i3 && (i3.next = o3), o3 && (o3.previous = i3), n3.first == r3 && (n3.first = o3), n3.last == r3 && (n3.last = i3), v ? n3.size-- : t3.size--;
    }
    return !!r3;
  }, forEach: function(e3) {
    for (var t3, n3 = c2(this), r3 = dr(e3, arguments.length > 1 ? arguments[1] : void 0); t3 = t3 ? t3.next : n3.first; )
      for (r3(t3.value, t3.key, this); t3 && t3.removed; )
        t3 = t3.previous;
  }, has: function(e3) {
    return !!s2(this, e3);
  } }), vc(i2, n2 ? { get: function(e3) {
    var t3 = s2(this, e3);
    return t3 && t3.value;
  }, set: function(e3, t3) {
    return a2(this, 0 === e3 ? 0 : e3, t3);
  } } : { add: function(e3) {
    return a2(this, e3 = 0 === e3 ? 0 : e3, e3);
  } }), v && bc(i2, "size", { get: function() {
    return c2(this).size;
  } }), o2;
}, setStrong: function(e2, t2, n2) {
  var r2 = t2 + " Iterator", o2 = Sc(t2), i2 = Sc(r2);
  Qo(e2, t2, function(e3, t3) {
    wc(this, { type: r2, target: e3, state: o2(e3), kind: t3, last: void 0 });
  }, function() {
    for (var e3 = i2(this), t3 = e3.kind, n3 = e3.last; n3 && n3.removed; )
      n3 = n3.previous;
    return e3.target && (e3.last = n3 = n3 ? n3.next : e3.state.first) ? qo("keys" == t3 ? n3.key : "values" == t3 ? n3.value : [n3.key, n3.value], false) : (e3.target = void 0, qo(void 0, true));
  }, n2 ? "entries" : "values", !n2, true), function(e3) {
    var t3 = z(e3), n3 = Je.f;
    v && t3 && !t3[mc] && n3(t3, mc, { configurable: true, get: function() {
      return this;
    } });
  }(t2);
} };
function _c(e2) {
  var t2 = this.constructor;
  return this.then(function(n2) {
    return t2.resolve(e2()).then(function() {
      return n2;
    });
  }, function(n2) {
    return t2.resolve(e2()).then(function() {
      return t2.reject(n2);
    });
  });
}
function Ic(e2) {
  return new this(function(t2, n2) {
    if (!e2 || void 0 === e2.length)
      return n2(new TypeError(typeof e2 + " " + e2 + " is not iterable(cannot read property Symbol(Symbol.iterator))"));
    var r2 = Array.prototype.slice.call(e2);
    if (0 === r2.length)
      return t2([]);
    var o2 = r2.length;
    function i2(e3, n3) {
      if (n3 && ("object" == typeof n3 || "function" == typeof n3)) {
        var c3 = n3.then;
        if ("function" == typeof c3)
          return void c3.call(n3, function(t3) {
            i2(e3, t3);
          }, function(n4) {
            r2[e3] = { status: "rejected", reason: n4 }, 0 == --o2 && t2(r2);
          });
      }
      r2[e3] = { status: "fulfilled", value: n3 }, 0 == --o2 && t2(r2);
    }
    for (var c2 = 0; c2 < r2.length; c2++)
      i2(c2, r2[c2]);
  });
}
!function(e2, t2, n2) {
  var r2 = -1 !== e2.indexOf("Map"), o2 = -1 !== e2.indexOf("Weak"), i2 = r2 ? "set" : "add", c2 = p[e2], a2 = c2 && c2.prototype, s2 = c2, u2 = {}, l2 = function(e3) {
    var t3 = L(a2[e3]);
    mt(a2, e3, "add" == e3 ? function(e4) {
      return t3(this, 0 === e4 ? 0 : e4), this;
    } : "delete" == e3 ? function(e4) {
      return !(o2 && !J(e4)) && t3(this, 0 === e4 ? 0 : e4);
    } : "get" == e3 ? function(e4) {
      return o2 && !J(e4) ? void 0 : t3(this, 0 === e4 ? 0 : e4);
    } : "has" == e3 ? function(e4) {
      return !(o2 && !J(e4)) && t3(this, 0 === e4 ? 0 : e4);
    } : function(e4, n3) {
      return t3(this, 0 === e4 ? 0 : e4, n3), this;
    });
  };
  if (Jt(e2, !H(c2) || !(o2 || a2.forEach && !y(function() {
    new c2().entries().next();
  }))))
    s2 = n2.getConstructor(t2, e2, r2, i2), uc.enable();
  else if (Jt(e2, true)) {
    var f2 = new s2(), d2 = f2[i2](o2 ? {} : -0, 1) != f2, h2 = y(function() {
      f2.has(1);
    }), v2 = vi(function(e3) {
      new c2(e3);
    }), m2 = !o2 && y(function() {
      for (var e3 = new c2(), t3 = 5; t3--; )
        e3[i2](t3, t3);
      return !e3.has(-0);
    });
    v2 || ((s2 = t2(function(e3, t3) {
      yc(e3, a2);
      var n3 = function(e4, t4, n4) {
        var r3, o3;
        return Ho && H(r3 = t4.constructor) && r3 !== n4 && J(o3 = r3.prototype) && o3 !== n4.prototype && Ho(e4, o3), e4;
      }(new c2(), e3, s2);
      return P(t3) || hc(t3, n3[i2], { that: n3, AS_ENTRIES: r2 }), n3;
    })).prototype = a2, a2.constructor = s2), (h2 || m2) && (l2("delete"), l2("has"), r2 && l2("get")), (m2 || d2) && l2(i2), o2 && a2.clear && delete a2.clear;
  }
  u2[e2] = s2, zt({ global: true, constructor: true, forced: s2 != c2 }, u2), lr(s2, e2), o2 || n2.setStrong(s2, e2, r2);
}("Set", function(e2) {
  return function() {
    return e2(this, arguments.length ? arguments[0] : void 0);
  };
}, kc), or.Set;
var Oc = setTimeout;
function xc(e2) {
  return Boolean(e2 && void 0 !== e2.length);
}
function Tc() {
}
function Cc(e2) {
  if (!(this instanceof Cc))
    throw new TypeError("Promises must be constructed via new");
  if ("function" != typeof e2)
    throw new TypeError("not a function");
  this._state = 0, this._handled = false, this._value = void 0, this._deferreds = [], Ec(e2, this);
}
function jc(e2, t2) {
  for (; 3 === e2._state; )
    e2 = e2._value;
  0 !== e2._state ? (e2._handled = true, Cc._immediateFn(function() {
    var n2 = 1 === e2._state ? t2.onFulfilled : t2.onRejected;
    if (null !== n2) {
      var r2;
      try {
        r2 = n2(e2._value);
      } catch (e3) {
        return void Rc(t2.promise, e3);
      }
      Lc(t2.promise, r2);
    } else
      (1 === e2._state ? Lc : Rc)(t2.promise, e2._value);
  })) : e2._deferreds.push(t2);
}
function Lc(e2, t2) {
  try {
    if (t2 === e2)
      throw new TypeError("A promise cannot be resolved with itself.");
    if (t2 && ("object" == typeof t2 || "function" == typeof t2)) {
      var n2 = t2.then;
      if (t2 instanceof Cc)
        return e2._state = 3, e2._value = t2, void Wc(e2);
      if ("function" == typeof n2)
        return void Ec((r2 = n2, o2 = t2, function() {
          r2.apply(o2, arguments);
        }), e2);
    }
    e2._state = 1, e2._value = t2, Wc(e2);
  } catch (t3) {
    Rc(e2, t3);
  }
  var r2, o2;
}
function Rc(e2, t2) {
  e2._state = 2, e2._value = t2, Wc(e2);
}
function Wc(e2) {
  2 === e2._state && 0 === e2._deferreds.length && Cc._immediateFn(function() {
    e2._handled || Cc._unhandledRejectionFn(e2._value);
  });
  for (var t2 = 0, n2 = e2._deferreds.length; t2 < n2; t2++)
    jc(e2, e2._deferreds[t2]);
  e2._deferreds = null;
}
function Zc(e2, t2, n2) {
  this.onFulfilled = "function" == typeof e2 ? e2 : null, this.onRejected = "function" == typeof t2 ? t2 : null, this.promise = n2;
}
function Ec(e2, t2) {
  var n2 = false;
  try {
    e2(function(e3) {
      n2 || (n2 = true, Lc(t2, e3));
    }, function(e3) {
      n2 || (n2 = true, Rc(t2, e3));
    });
  } catch (e3) {
    if (n2)
      return;
    n2 = true, Rc(t2, e3);
  }
}
Cc.prototype.catch = function(e2) {
  return this.then(null, e2);
}, Cc.prototype.then = function(e2, t2) {
  var n2 = new this.constructor(Tc);
  return jc(this, new Zc(e2, t2, n2)), n2;
}, Cc.prototype.finally = _c, Cc.all = function(e2) {
  return new Cc(function(t2, n2) {
    if (!xc(e2))
      return n2(new TypeError("Promise.all accepts an array"));
    var r2 = Array.prototype.slice.call(e2);
    if (0 === r2.length)
      return t2([]);
    var o2 = r2.length;
    function i2(e3, c3) {
      try {
        if (c3 && ("object" == typeof c3 || "function" == typeof c3)) {
          var a2 = c3.then;
          if ("function" == typeof a2)
            return void a2.call(c3, function(t3) {
              i2(e3, t3);
            }, n2);
        }
        r2[e3] = c3, 0 == --o2 && t2(r2);
      } catch (e4) {
        n2(e4);
      }
    }
    for (var c2 = 0; c2 < r2.length; c2++)
      i2(c2, r2[c2]);
  });
}, Cc.allSettled = Ic, Cc.resolve = function(e2) {
  return e2 && "object" == typeof e2 && e2.constructor === Cc ? e2 : new Cc(function(t2) {
    t2(e2);
  });
}, Cc.reject = function(e2) {
  return new Cc(function(t2, n2) {
    n2(e2);
  });
}, Cc.race = function(e2) {
  return new Cc(function(t2, n2) {
    if (!xc(e2))
      return n2(new TypeError("Promise.race accepts an array"));
    for (var r2 = 0, o2 = e2.length; r2 < o2; r2++)
      Cc.resolve(e2[r2]).then(t2, n2);
  });
}, Cc._immediateFn = "function" == typeof setImmediate && function(e2) {
  setImmediate(e2);
} || function(e2) {
  Oc(e2, 0);
}, Cc._unhandledRejectionFn = function(e2) {
  "undefined" != typeof console && console && console.warn("Possible Unhandled Promise Rejection:", e2);
};
var Gc = function() {
  if ("undefined" != typeof self)
    return self;
  if ("undefined" != typeof window)
    return window;
  if ("undefined" != typeof global)
    return global;
  throw new Error("unable to locate global object");
}();
"function" != typeof Gc.Promise ? Gc.Promise = Cc : (Gc.Promise.prototype.finally || (Gc.Promise.prototype.finally = _c), Gc.Promise.allSettled || (Gc.Promise.allSettled = Ic)), function(e2) {
  function t2(e3) {
    for (var t3 = 0, n3 = Math.min(65536, e3.length + 1), r3 = new Uint16Array(n3), o3 = [], i3 = 0; ; ) {
      var c3 = t3 < e3.length;
      if (!c3 || i3 >= n3 - 1) {
        var a3 = r3.subarray(0, i3);
        if (o3.push(String.fromCharCode.apply(null, a3)), !c3)
          return o3.join("");
        e3 = e3.subarray(t3), t3 = 0, i3 = 0;
      }
      var s3 = e3[t3++];
      if (0 == (128 & s3))
        r3[i3++] = s3;
      else if (192 == (224 & s3)) {
        var u3 = 63 & e3[t3++];
        r3[i3++] = (31 & s3) << 6 | u3;
      } else if (224 == (240 & s3)) {
        u3 = 63 & e3[t3++];
        var l3 = 63 & e3[t3++];
        r3[i3++] = (31 & s3) << 12 | u3 << 6 | l3;
      } else if (240 == (248 & s3)) {
        var f3 = (7 & s3) << 18 | (u3 = 63 & e3[t3++]) << 12 | (l3 = 63 & e3[t3++]) << 6 | 63 & e3[t3++];
        f3 > 65535 && (f3 -= 65536, r3[i3++] = f3 >>> 10 & 1023 | 55296, f3 = 56320 | 1023 & f3), r3[i3++] = f3;
      }
    }
  }
  var n2 = "Failed to ", r2 = function(e3, t3, r3) {
    if (e3)
      throw new Error("".concat(n2).concat(t3, ": the '").concat(r3, "' option is unsupported."));
  }, o2 = "function" == typeof Buffer && Buffer.from, i2 = o2 ? function(e3) {
    return Buffer.from(e3);
  } : function(e3) {
    for (var t3 = 0, n3 = e3.length, r3 = 0, o3 = Math.max(32, n3 + (n3 >>> 1) + 7), i3 = new Uint8Array(o3 >>> 3 << 3); t3 < n3; ) {
      var c3 = e3.charCodeAt(t3++);
      if (c3 >= 55296 && c3 <= 56319) {
        if (t3 < n3) {
          var a3 = e3.charCodeAt(t3);
          56320 == (64512 & a3) && (++t3, c3 = ((1023 & c3) << 10) + (1023 & a3) + 65536);
        }
        if (c3 >= 55296 && c3 <= 56319)
          continue;
      }
      if (r3 + 4 > i3.length) {
        o3 += 8, o3 = (o3 *= 1 + t3 / e3.length * 2) >>> 3 << 3;
        var s3 = new Uint8Array(o3);
        s3.set(i3), i3 = s3;
      }
      if (0 != (4294967168 & c3)) {
        if (0 == (4294965248 & c3))
          i3[r3++] = c3 >>> 6 & 31 | 192;
        else if (0 == (4294901760 & c3))
          i3[r3++] = c3 >>> 12 & 15 | 224, i3[r3++] = c3 >>> 6 & 63 | 128;
        else {
          if (0 != (4292870144 & c3))
            continue;
          i3[r3++] = c3 >>> 18 & 7 | 240, i3[r3++] = c3 >>> 12 & 63 | 128, i3[r3++] = c3 >>> 6 & 63 | 128;
        }
        i3[r3++] = 63 & c3 | 128;
      } else
        i3[r3++] = c3;
    }
    return i3.slice ? i3.slice(0, r3) : i3.subarray(0, r3);
  };
  function c2() {
    this.encoding = "utf-8";
  }
  c2.prototype.encode = function(e3, t3) {
    return r2(t3 && t3.stream, "encode", "stream"), i2(e3);
  };
  var a2 = !o2 && "function" == typeof Blob && "function" == typeof URL && "function" == typeof URL.createObjectURL, s2 = ["utf-8", "utf8", "unicode-1-1-utf-8"], u2 = t2;
  o2 ? u2 = function(e3, t3) {
    return (e3 instanceof Buffer ? e3 : Buffer.from(e3.buffer, e3.byteOffset, e3.byteLength)).toString(t3);
  } : a2 && (u2 = function(e3) {
    try {
      return function(e4) {
        var t3;
        try {
          var n3 = new Blob([e4], { type: "text/plain;charset=UTF-8" });
          t3 = URL.createObjectURL(n3);
          var r3 = new XMLHttpRequest();
          return r3.open("GET", t3, false), r3.send(), r3.responseText;
        } finally {
          t3 && URL.revokeObjectURL(t3);
        }
      }(e3);
    } catch (n3) {
      return t2(e3);
    }
  });
  var l2 = "construct 'TextDecoder'", f2 = "".concat(n2, " ").concat(l2, ": the ");
  function d2(e3, t3) {
    if (r2(t3 && t3.fatal, l2, "fatal"), e3 = e3 || "utf-8", !(o2 ? Buffer.isEncoding(e3) : -1 !== s2.indexOf(e3.toLowerCase())))
      throw new RangeError("".concat(f2, " encoding label provided ('").concat(e3, "') is invalid."));
    this.encoding = e3, this.fatal = false, this.ignoreBOM = false;
  }
  d2.prototype.decode = function(e3, t3) {
    var n3;
    return r2(t3 && t3.stream, "decode", "stream"), n3 = e3 instanceof Uint8Array ? e3 : e3.buffer instanceof ArrayBuffer ? new Uint8Array(e3.buffer) : new Uint8Array(e3), u2(n3, this.encoding);
  }, e2.TextEncoder = e2.TextEncoder || c2, e2.TextDecoder = e2.TextDecoder || d2;
}("undefined" != typeof window ? window : s), function() {
  function e2(e3, t3) {
    if (!(e3 instanceof t3))
      throw new TypeError("Cannot call a class as a function");
  }
  function t2(e3, t3) {
    for (var n3 = 0; n3 < t3.length; n3++) {
      var r3 = t3[n3];
      r3.enumerable = r3.enumerable || false, r3.configurable = true, "value" in r3 && (r3.writable = true), Object.defineProperty(e3, r3.key, r3);
    }
  }
  function n2(e3, n3, r3) {
    return n3 && t2(e3.prototype, n3), r3 && t2(e3, r3), e3;
  }
  function r2(e3, t3) {
    if ("function" != typeof t3 && null !== t3)
      throw new TypeError("Super expression must either be null or a function");
    e3.prototype = Object.create(t3 && t3.prototype, { constructor: { value: e3, writable: true, configurable: true } }), t3 && i2(e3, t3);
  }
  function o2(e3) {
    return o2 = Object.setPrototypeOf ? Object.getPrototypeOf : function(e4) {
      return e4.__proto__ || Object.getPrototypeOf(e4);
    }, o2(e3);
  }
  function i2(e3, t3) {
    return i2 = Object.setPrototypeOf || function(e4, t4) {
      return e4.__proto__ = t4, e4;
    }, i2(e3, t3);
  }
  function c2() {
    if ("undefined" == typeof Reflect || !Reflect.construct)
      return false;
    if (Reflect.construct.sham)
      return false;
    if ("function" == typeof Proxy)
      return true;
    try {
      return Boolean.prototype.valueOf.call(Reflect.construct(Boolean, [], function() {
      })), true;
    } catch (e3) {
      return false;
    }
  }
  function a2(e3) {
    if (void 0 === e3)
      throw new ReferenceError("this hasn't been initialised - super() hasn't been called");
    return e3;
  }
  function u2(e3, t3) {
    return !t3 || "object" != typeof t3 && "function" != typeof t3 ? a2(e3) : t3;
  }
  function l2(e3) {
    var t3 = c2();
    return function() {
      var n3, r3 = o2(e3);
      if (t3) {
        var i3 = o2(this).constructor;
        n3 = Reflect.construct(r3, arguments, i3);
      } else
        n3 = r3.apply(this, arguments);
      return u2(this, n3);
    };
  }
  function f2(e3, t3) {
    for (; !Object.prototype.hasOwnProperty.call(e3, t3) && null !== (e3 = o2(e3)); )
      ;
    return e3;
  }
  function d2(e3, t3, n3) {
    return d2 = "undefined" != typeof Reflect && Reflect.get ? Reflect.get : function(e4, t4, n4) {
      var r3 = f2(e4, t4);
      if (r3) {
        var o3 = Object.getOwnPropertyDescriptor(r3, t4);
        return o3.get ? o3.get.call(n4) : o3.value;
      }
    }, d2(e3, t3, n3 || e3);
  }
  var h2 = function() {
    function t3() {
      e2(this, t3), Object.defineProperty(this, "listeners", { value: {}, writable: true, configurable: true });
    }
    return n2(t3, [{ key: "addEventListener", value: function(e3, t4, n3) {
      e3 in this.listeners || (this.listeners[e3] = []), this.listeners[e3].push({ callback: t4, options: n3 });
    } }, { key: "removeEventListener", value: function(e3, t4) {
      if (e3 in this.listeners) {
        for (var n3 = this.listeners[e3], r3 = 0, o3 = n3.length; r3 < o3; r3++)
          if (n3[r3].callback === t4)
            return void n3.splice(r3, 1);
      }
    } }, { key: "dispatchEvent", value: function(e3) {
      if (e3.type in this.listeners) {
        for (var t4 = this.listeners[e3.type].slice(), n3 = 0, r3 = t4.length; n3 < r3; n3++) {
          var o3 = t4[n3];
          try {
            o3.callback.call(this, e3);
          } catch (e4) {
            Promise.resolve().then(function() {
              throw e4;
            });
          }
          o3.options && o3.options.once && this.removeEventListener(e3.type, o3.callback);
        }
        return !e3.defaultPrevented;
      }
    } }]), t3;
  }(), p2 = function(t3) {
    r2(c3, t3);
    var i3 = l2(c3);
    function c3() {
      var t4;
      return e2(this, c3), (t4 = i3.call(this)).listeners || h2.call(a2(t4)), Object.defineProperty(a2(t4), "aborted", { value: false, writable: true, configurable: true }), Object.defineProperty(a2(t4), "onabort", { value: null, writable: true, configurable: true }), t4;
    }
    return n2(c3, [{ key: "toString", value: function() {
      return "[object AbortSignal]";
    } }, { key: "dispatchEvent", value: function(e3) {
      "abort" === e3.type && (this.aborted = true, "function" == typeof this.onabort && this.onabort.call(this, e3)), d2(o2(c3.prototype), "dispatchEvent", this).call(this, e3);
    } }]), c3;
  }(h2), y2 = function() {
    function t3() {
      e2(this, t3), Object.defineProperty(this, "signal", { value: new p2(), writable: true, configurable: true });
    }
    return n2(t3, [{ key: "abort", value: function() {
      var e3;
      try {
        e3 = new Event("abort");
      } catch (t4) {
        "undefined" != typeof document ? document.createEvent ? (e3 = document.createEvent("Event")).initEvent("abort", false, false) : (e3 = document.createEventObject()).type = "abort" : e3 = { type: "abort", bubbles: false, cancelable: false };
      }
      this.signal.dispatchEvent(e3);
    } }, { key: "toString", value: function() {
      return "[object AbortController]";
    } }]), t3;
  }();
  function v2(e3) {
    return e3.__FORCE_INSTALL_ABORTCONTROLLER_POLYFILL ? (console.log("__FORCE_INSTALL_ABORTCONTROLLER_POLYFILL=true is set, will force install polyfill"), true) : "function" == typeof e3.Request && !e3.Request.prototype.hasOwnProperty("signal") || !e3.AbortController;
  }
  "undefined" != typeof Symbol && Symbol.toStringTag && (y2.prototype[Symbol.toStringTag] = "AbortController", p2.prototype[Symbol.toStringTag] = "AbortSignal"), function(e3) {
    v2(e3) && (e3.AbortController = y2, e3.AbortSignal = p2);
  }("undefined" != typeof self ? self : s);
}();
var Ac = l(function(e2, t2) {
  Object.defineProperty(t2, "__esModule", { value: true });
  var n2 = function() {
    function e3() {
      var e4 = this;
      this.locked = /* @__PURE__ */ new Map(), this.addToLocked = function(t3, n3) {
        var r2 = e4.locked.get(t3);
        void 0 === r2 ? void 0 === n3 ? e4.locked.set(t3, []) : e4.locked.set(t3, [n3]) : void 0 !== n3 && (r2.unshift(n3), e4.locked.set(t3, r2));
      }, this.isLocked = function(t3) {
        return e4.locked.has(t3);
      }, this.lock = function(t3) {
        return new Promise(function(n3, r2) {
          e4.isLocked(t3) ? e4.addToLocked(t3, n3) : (e4.addToLocked(t3), n3());
        });
      }, this.unlock = function(t3) {
        var n3 = e4.locked.get(t3);
        if (void 0 !== n3 && 0 !== n3.length) {
          var r2 = n3.pop();
          e4.locked.set(t3, n3), void 0 !== r2 && setTimeout(r2, 0);
        } else
          e4.locked.delete(t3);
      };
    }
    return e3.getInstance = function() {
      return void 0 === e3.instance && (e3.instance = new e3()), e3.instance;
    }, e3;
  }();
  t2.default = function() {
    return n2.getInstance();
  };
});
u(Ac);
var Pc = l(function(e2, t2) {
  var n2 = s && s.__awaiter || function(e3, t3, n3, r3) {
    return new (n3 || (n3 = Promise))(function(o3, i3) {
      function c3(e4) {
        try {
          s2(r3.next(e4));
        } catch (e5) {
          i3(e5);
        }
      }
      function a3(e4) {
        try {
          s2(r3.throw(e4));
        } catch (e5) {
          i3(e5);
        }
      }
      function s2(e4) {
        e4.done ? o3(e4.value) : new n3(function(t4) {
          t4(e4.value);
        }).then(c3, a3);
      }
      s2((r3 = r3.apply(e3, t3 || [])).next());
    });
  }, r2 = s && s.__generator || function(e3, t3) {
    var n3, r3, o3, i3, c3 = { label: 0, sent: function() {
      if (1 & o3[0])
        throw o3[1];
      return o3[1];
    }, trys: [], ops: [] };
    return i3 = { next: a3(0), throw: a3(1), return: a3(2) }, "function" == typeof Symbol && (i3[Symbol.iterator] = function() {
      return this;
    }), i3;
    function a3(i4) {
      return function(a4) {
        return function(i5) {
          if (n3)
            throw new TypeError("Generator is already executing.");
          for (; c3; )
            try {
              if (n3 = 1, r3 && (o3 = 2 & i5[0] ? r3.return : i5[0] ? r3.throw || ((o3 = r3.return) && o3.call(r3), 0) : r3.next) && !(o3 = o3.call(r3, i5[1])).done)
                return o3;
              switch (r3 = 0, o3 && (i5 = [2 & i5[0], o3.value]), i5[0]) {
                case 0:
                case 1:
                  o3 = i5;
                  break;
                case 4:
                  return c3.label++, { value: i5[1], done: false };
                case 5:
                  c3.label++, r3 = i5[1], i5 = [0];
                  continue;
                case 7:
                  i5 = c3.ops.pop(), c3.trys.pop();
                  continue;
                default:
                  if (!(o3 = c3.trys, (o3 = o3.length > 0 && o3[o3.length - 1]) || 6 !== i5[0] && 2 !== i5[0])) {
                    c3 = 0;
                    continue;
                  }
                  if (3 === i5[0] && (!o3 || i5[1] > o3[0] && i5[1] < o3[3])) {
                    c3.label = i5[1];
                    break;
                  }
                  if (6 === i5[0] && c3.label < o3[1]) {
                    c3.label = o3[1], o3 = i5;
                    break;
                  }
                  if (o3 && c3.label < o3[2]) {
                    c3.label = o3[2], c3.ops.push(i5);
                    break;
                  }
                  o3[2] && c3.ops.pop(), c3.trys.pop();
                  continue;
              }
              i5 = t3.call(e3, c3);
            } catch (e4) {
              i5 = [6, e4], r3 = 0;
            } finally {
              n3 = o3 = 0;
            }
          if (5 & i5[0])
            throw i5[1];
          return { value: i5[0] ? i5[1] : void 0, done: true };
        }([i4, a4]);
      };
    }
  };
  Object.defineProperty(t2, "__esModule", { value: true });
  var o2 = "browser-tabs-lock-key";
  function i2(e3) {
    return new Promise(function(t3) {
      return setTimeout(t3, e3);
    });
  }
  function c2(e3) {
    for (var t3 = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXTZabcdefghiklmnopqrstuvwxyz", n3 = "", r3 = 0; r3 < e3; r3++) {
      n3 += t3[Math.floor(Math.random() * t3.length)];
    }
    return n3;
  }
  var a2 = function() {
    function e3() {
      this.acquiredIatSet = /* @__PURE__ */ new Set(), this.id = Date.now().toString() + c2(15), this.acquireLock = this.acquireLock.bind(this), this.releaseLock = this.releaseLock.bind(this), this.releaseLock__private__ = this.releaseLock__private__.bind(this), this.waitForSomethingToChange = this.waitForSomethingToChange.bind(this), this.refreshLockWhileAcquired = this.refreshLockWhileAcquired.bind(this), void 0 === e3.waiters && (e3.waiters = []);
    }
    return e3.prototype.acquireLock = function(t3, a3) {
      return void 0 === a3 && (a3 = 5e3), n2(this, void 0, void 0, function() {
        var n3, s2, u2, l2, f2, d2;
        return r2(this, function(r3) {
          switch (r3.label) {
            case 0:
              n3 = Date.now() + c2(4), s2 = Date.now() + a3, u2 = o2 + "-" + t3, l2 = window.localStorage, r3.label = 1;
            case 1:
              return Date.now() < s2 ? [4, i2(30)] : [3, 8];
            case 2:
              return r3.sent(), null !== l2.getItem(u2) ? [3, 5] : (f2 = this.id + "-" + t3 + "-" + n3, [4, i2(Math.floor(25 * Math.random()))]);
            case 3:
              return r3.sent(), l2.setItem(u2, JSON.stringify({ id: this.id, iat: n3, timeoutKey: f2, timeAcquired: Date.now(), timeRefreshed: Date.now() })), [4, i2(30)];
            case 4:
              return r3.sent(), null !== (d2 = l2.getItem(u2)) && (d2 = JSON.parse(d2)).id === this.id && d2.iat === n3 ? (this.acquiredIatSet.add(n3), this.refreshLockWhileAcquired(u2, n3), [2, true]) : [3, 7];
            case 5:
              return e3.lockCorrector(), [4, this.waitForSomethingToChange(s2)];
            case 6:
              r3.sent(), r3.label = 7;
            case 7:
              return n3 = Date.now() + c2(4), [3, 1];
            case 8:
              return [2, false];
          }
        });
      });
    }, e3.prototype.refreshLockWhileAcquired = function(e4, t3) {
      return n2(this, void 0, void 0, function() {
        var o3 = this;
        return r2(this, function(i3) {
          return setTimeout(function() {
            return n2(o3, void 0, void 0, function() {
              var n3, o4;
              return r2(this, function(r3) {
                switch (r3.label) {
                  case 0:
                    return [4, Ac.default().lock(t3)];
                  case 1:
                    return r3.sent(), this.acquiredIatSet.has(t3) ? (n3 = window.localStorage, null === (o4 = n3.getItem(e4)) ? (Ac.default().unlock(t3), [2]) : ((o4 = JSON.parse(o4)).timeRefreshed = Date.now(), n3.setItem(e4, JSON.stringify(o4)), Ac.default().unlock(t3), this.refreshLockWhileAcquired(e4, t3), [2])) : (Ac.default().unlock(t3), [2]);
                }
              });
            });
          }, 1e3), [2];
        });
      });
    }, e3.prototype.waitForSomethingToChange = function(t3) {
      return n2(this, void 0, void 0, function() {
        return r2(this, function(n3) {
          switch (n3.label) {
            case 0:
              return [4, new Promise(function(n4) {
                var r3 = false, o3 = Date.now(), i3 = false;
                function c3() {
                  if (i3 || (window.removeEventListener("storage", c3), e3.removeFromWaiting(c3), clearTimeout(a3), i3 = true), !r3) {
                    r3 = true;
                    var t4 = 50 - (Date.now() - o3);
                    t4 > 0 ? setTimeout(n4, t4) : n4();
                  }
                }
                window.addEventListener("storage", c3), e3.addToWaiting(c3);
                var a3 = setTimeout(c3, Math.max(0, t3 - Date.now()));
              })];
            case 1:
              return n3.sent(), [2];
          }
        });
      });
    }, e3.addToWaiting = function(t3) {
      this.removeFromWaiting(t3), void 0 !== e3.waiters && e3.waiters.push(t3);
    }, e3.removeFromWaiting = function(t3) {
      void 0 !== e3.waiters && (e3.waiters = e3.waiters.filter(function(e4) {
        return e4 !== t3;
      }));
    }, e3.notifyWaiters = function() {
      void 0 !== e3.waiters && e3.waiters.slice().forEach(function(e4) {
        return e4();
      });
    }, e3.prototype.releaseLock = function(e4) {
      return n2(this, void 0, void 0, function() {
        return r2(this, function(t3) {
          switch (t3.label) {
            case 0:
              return [4, this.releaseLock__private__(e4)];
            case 1:
              return [2, t3.sent()];
          }
        });
      });
    }, e3.prototype.releaseLock__private__ = function(t3) {
      return n2(this, void 0, void 0, function() {
        var n3, i3, c3;
        return r2(this, function(r3) {
          switch (r3.label) {
            case 0:
              return n3 = window.localStorage, i3 = o2 + "-" + t3, null === (c3 = n3.getItem(i3)) ? [2] : (c3 = JSON.parse(c3)).id !== this.id ? [3, 2] : [4, Ac.default().lock(c3.iat)];
            case 1:
              r3.sent(), this.acquiredIatSet.delete(c3.iat), n3.removeItem(i3), Ac.default().unlock(c3.iat), e3.notifyWaiters(), r3.label = 2;
            case 2:
              return [2];
          }
        });
      });
    }, e3.lockCorrector = function() {
      for (var t3 = Date.now() - 5e3, n3 = window.localStorage, r3 = Object.keys(n3), i3 = false, c3 = 0; c3 < r3.length; c3++) {
        var a3 = r3[c3];
        if (a3.includes(o2)) {
          var s2 = n3.getItem(a3);
          null !== s2 && (void 0 === (s2 = JSON.parse(s2)).timeRefreshed && s2.timeAcquired < t3 || void 0 !== s2.timeRefreshed && s2.timeRefreshed < t3) && (n3.removeItem(a3), i3 = true);
        }
      }
      i3 && e3.notifyWaiters();
    }, e3.waiters = void 0, e3;
  }();
  t2.default = a2;
});
var Xc = u(Pc);
var Fc = { timeoutInSeconds: 60 };
var Kc = ["login_required", "consent_required", "interaction_required", "account_selection_required", "access_denied"];
var Nc = { name: "auth0-spa-js", version: "1.22.6" };
var Uc = function() {
  return Date.now();
};
var Dc = function(e2) {
  function n2(t2, r2) {
    var o2 = e2.call(this, r2) || this;
    return o2.error = t2, o2.error_description = r2, Object.setPrototypeOf(o2, n2.prototype), o2;
  }
  return t(n2, e2), n2.fromPayload = function(e3) {
    return new n2(e3.error, e3.error_description);
  }, n2;
}(Error);
var Hc = function(e2) {
  function n2(t2, r2, o2, i2) {
    void 0 === i2 && (i2 = null);
    var c2 = e2.call(this, t2, r2) || this;
    return c2.state = o2, c2.appState = i2, Object.setPrototypeOf(c2, n2.prototype), c2;
  }
  return t(n2, e2), n2;
}(Dc);
var Yc = function(e2) {
  function n2() {
    var t2 = e2.call(this, "timeout", "Timeout") || this;
    return Object.setPrototypeOf(t2, n2.prototype), t2;
  }
  return t(n2, e2), n2;
}(Dc);
var Jc = function(e2) {
  function n2(t2) {
    var r2 = e2.call(this) || this;
    return r2.popup = t2, Object.setPrototypeOf(r2, n2.prototype), r2;
  }
  return t(n2, e2), n2;
}(Yc);
var Vc = function(e2) {
  function n2(t2) {
    var r2 = e2.call(this, "cancelled", "Popup closed") || this;
    return r2.popup = t2, Object.setPrototypeOf(r2, n2.prototype), r2;
  }
  return t(n2, e2), n2;
}(Dc);
var zc = function(e2) {
  function n2(t2, r2, o2) {
    var i2 = e2.call(this, t2, r2) || this;
    return i2.mfa_token = o2, Object.setPrototypeOf(i2, n2.prototype), i2;
  }
  return t(n2, e2), n2;
}(Dc);
var Mc = function(e2) {
  function n2(t2, r2) {
    var o2 = e2.call(this, "missing_refresh_token", "Missing Refresh Token (audience: '".concat(ia(t2, ["default"]), "', scope: '").concat(ia(r2), "')")) || this;
    return o2.audience = t2, o2.scope = r2, Object.setPrototypeOf(o2, n2.prototype), o2;
  }
  return t(n2, e2), n2;
}(Dc);
var Bc = function(e2) {
  return new Promise(function(t2, n2) {
    var r2, o2 = setInterval(function() {
      e2.popup && e2.popup.closed && (clearInterval(o2), clearTimeout(i2), window.removeEventListener("message", r2, false), n2(new Vc(e2.popup)));
    }, 1e3), i2 = setTimeout(function() {
      clearInterval(o2), n2(new Jc(e2.popup)), window.removeEventListener("message", r2, false);
    }, 1e3 * (e2.timeoutInSeconds || 60));
    r2 = function(c2) {
      if (c2.data && "authorization_response" === c2.data.type) {
        if (clearTimeout(i2), clearInterval(o2), window.removeEventListener("message", r2, false), e2.popup.close(), c2.data.response.error)
          return n2(Dc.fromPayload(c2.data.response));
        t2(c2.data.response);
      }
    }, window.addEventListener("message", r2);
  });
};
var Qc = function() {
  return window.crypto || window.msCrypto;
};
var qc = function() {
  var e2 = Qc();
  return e2.subtle || e2.webkitSubtle;
};
var $c = function() {
  var e2 = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz-_~.", t2 = "";
  return Array.from(Qc().getRandomValues(new Uint8Array(43))).forEach(function(n2) {
    return t2 += e2[n2 % e2.length];
  }), t2;
};
var ea = function(e2) {
  return btoa(e2);
};
var ta = function(e2) {
  return Object.keys(e2).filter(function(t2) {
    return void 0 !== e2[t2];
  }).map(function(t2) {
    return encodeURIComponent(t2) + "=" + encodeURIComponent(e2[t2]);
  }).join("&");
};
var na = function(e2) {
  return o(void 0, void 0, void 0, function() {
    var t2;
    return i(this, function(n2) {
      switch (n2.label) {
        case 0:
          return t2 = qc().digest({ name: "SHA-256" }, new TextEncoder().encode(e2)), window.msCrypto ? [2, new Promise(function(e3, n3) {
            t2.oncomplete = function(t3) {
              e3(t3.target.result);
            }, t2.onerror = function(e4) {
              n3(e4.error);
            }, t2.onabort = function() {
              n3("The digest operation was aborted");
            };
          })] : [4, t2];
        case 1:
          return [2, n2.sent()];
      }
    });
  });
};
var ra = function(e2) {
  return function(e3) {
    return decodeURIComponent(atob(e3).split("").map(function(e4) {
      return "%" + ("00" + e4.charCodeAt(0).toString(16)).slice(-2);
    }).join(""));
  }(e2.replace(/_/g, "/").replace(/-/g, "+"));
};
var oa = function(e2) {
  var t2 = new Uint8Array(e2);
  return function(e3) {
    var t3 = { "+": "-", "/": "_", "=": "" };
    return e3.replace(/[+/=]/g, function(e4) {
      return t3[e4];
    });
  }(window.btoa(String.fromCharCode.apply(String, a([], c(Array.from(t2)), false))));
};
function ia(e2, t2) {
  return void 0 === t2 && (t2 = []), e2 && !t2.includes(e2) ? e2 : "";
}
var ca = function(e2, t2) {
  return o(void 0, void 0, void 0, function() {
    var n2, r2;
    return i(this, function(o2) {
      switch (o2.label) {
        case 0:
          return [4, (i2 = e2, c2 = t2, c2 = c2 || {}, new Promise(function(e3, t3) {
            var n3 = new XMLHttpRequest(), r3 = [], o3 = [], a2 = {}, s2 = function() {
              return { ok: 2 == (n3.status / 100 | 0), statusText: n3.statusText, status: n3.status, url: n3.responseURL, text: function() {
                return Promise.resolve(n3.responseText);
              }, json: function() {
                return Promise.resolve(n3.responseText).then(JSON.parse);
              }, blob: function() {
                return Promise.resolve(new Blob([n3.response]));
              }, clone: s2, headers: { keys: function() {
                return r3;
              }, entries: function() {
                return o3;
              }, get: function(e4) {
                return a2[e4.toLowerCase()];
              }, has: function(e4) {
                return e4.toLowerCase() in a2;
              } } };
            };
            for (var u2 in n3.open(c2.method || "get", i2, true), n3.onload = function() {
              n3.getAllResponseHeaders().replace(/^(.*?):[^\S\n]*([\s\S]*?)$/gm, function(e4, t4, n4) {
                r3.push(t4 = t4.toLowerCase()), o3.push([t4, n4]), a2[t4] = a2[t4] ? a2[t4] + "," + n4 : n4;
              }), e3(s2());
            }, n3.onerror = t3, n3.withCredentials = "include" == c2.credentials, c2.headers)
              n3.setRequestHeader(u2, c2.headers[u2]);
            n3.send(c2.body || null);
          }))];
        case 1:
          return n2 = o2.sent(), r2 = { ok: n2.ok }, [4, n2.json()];
        case 2:
          return [2, (r2.json = o2.sent(), r2)];
      }
      var i2, c2;
    });
  });
};
var aa = function(e2, t2, n2) {
  return o(void 0, void 0, void 0, function() {
    var r2, o2;
    return i(this, function(i2) {
      return r2 = new AbortController(), t2.signal = r2.signal, [2, Promise.race([ca(e2, t2), new Promise(function(e3, t3) {
        o2 = setTimeout(function() {
          r2.abort(), t3(new Error("Timeout when executing 'fetch'"));
        }, n2);
      })]).finally(function() {
        clearTimeout(o2);
      })];
    });
  });
};
var sa = function(e2, t2, n2, r2, c2, a2, s2) {
  return o(void 0, void 0, void 0, function() {
    return i(this, function(o2) {
      return [2, (i2 = { auth: { audience: t2, scope: n2 }, timeout: c2, fetchUrl: e2, fetchOptions: r2, useFormData: s2 }, u2 = a2, new Promise(function(e3, t3) {
        var n3 = new MessageChannel();
        n3.port1.onmessage = function(r3) {
          r3.data.error ? t3(new Error(r3.data.error)) : e3(r3.data), n3.port1.close();
        }, u2.postMessage(i2, [n3.port2]);
      }))];
      var i2, u2;
    });
  });
};
var ua = function(e2, t2, n2, r2, c2, a2, s2) {
  return void 0 === s2 && (s2 = 1e4), o(void 0, void 0, void 0, function() {
    return i(this, function(o2) {
      return c2 ? [2, sa(e2, t2, n2, r2, s2, c2, a2)] : [2, aa(e2, r2, s2)];
    });
  });
};
function la(e2, t2, n2, c2, a2, s2, u2) {
  return o(this, void 0, void 0, function() {
    var o2, l2, f2, d2, h2, p2, y2, v2, m2;
    return i(this, function(i2) {
      switch (i2.label) {
        case 0:
          o2 = null, f2 = 0, i2.label = 1;
        case 1:
          if (!(f2 < 3))
            return [3, 6];
          i2.label = 2;
        case 2:
          return i2.trys.push([2, 4, , 5]), [4, ua(e2, n2, c2, a2, s2, u2, t2)];
        case 3:
          return l2 = i2.sent(), o2 = null, [3, 6];
        case 4:
          return d2 = i2.sent(), o2 = d2, [3, 5];
        case 5:
          return f2++, [3, 1];
        case 6:
          if (o2)
            throw o2.message = o2.message || "Failed to fetch", o2;
          if (h2 = l2.json, p2 = h2.error, y2 = h2.error_description, v2 = r(h2, ["error", "error_description"]), !l2.ok) {
            if (m2 = y2 || "HTTP error. Unable to fetch ".concat(e2), "mfa_required" === p2)
              throw new zc(p2, m2, v2.mfa_token);
            throw new Dc(p2 || "request_error", m2);
          }
          return [2, v2];
      }
    });
  });
}
function fa(e2, t2) {
  var n2 = e2.baseUrl, c2 = e2.timeout, a2 = e2.audience, s2 = e2.scope, u2 = e2.auth0Client, l2 = e2.useFormData, f2 = r(e2, ["baseUrl", "timeout", "audience", "scope", "auth0Client", "useFormData"]);
  return o(this, void 0, void 0, function() {
    var e3;
    return i(this, function(r2) {
      switch (r2.label) {
        case 0:
          return e3 = l2 ? ta(f2) : JSON.stringify(f2), [4, la("".concat(n2, "/oauth/token"), c2, a2 || "default", s2, { method: "POST", body: e3, headers: { "Content-Type": l2 ? "application/x-www-form-urlencoded" : "application/json", "Auth0-Client": btoa(JSON.stringify(u2 || Nc)) } }, t2, l2)];
        case 1:
          return [2, r2.sent()];
      }
    });
  });
}
var da = function(e2) {
  return Array.from(new Set(e2));
};
var ha = function() {
  for (var e2 = [], t2 = 0; t2 < arguments.length; t2++)
    e2[t2] = arguments[t2];
  return da(e2.join(" ").trim().split(/\s+/)).join(" ");
};
var pa = function() {
  function e2(e3, t2) {
    void 0 === t2 && (t2 = "@@auth0spajs@@"), this.prefix = t2, this.client_id = e3.client_id, this.scope = e3.scope, this.audience = e3.audience;
  }
  return e2.prototype.toKey = function() {
    return "".concat(this.prefix, "::").concat(this.client_id, "::").concat(this.audience, "::").concat(this.scope);
  }, e2.fromKey = function(t2) {
    var n2 = c(t2.split("::"), 4), r2 = n2[0], o2 = n2[1], i2 = n2[2];
    return new e2({ client_id: o2, scope: n2[3], audience: i2 }, r2);
  }, e2.fromCacheEntry = function(t2) {
    return new e2({ scope: t2.scope, audience: t2.audience, client_id: t2.client_id });
  }, e2;
}();
var ya = function() {
  function e2() {
  }
  return e2.prototype.set = function(e3, t2) {
    localStorage.setItem(e3, JSON.stringify(t2));
  }, e2.prototype.get = function(e3) {
    var t2 = window.localStorage.getItem(e3);
    if (t2)
      try {
        return JSON.parse(t2);
      } catch (e4) {
        return;
      }
  }, e2.prototype.remove = function(e3) {
    localStorage.removeItem(e3);
  }, e2.prototype.allKeys = function() {
    return Object.keys(window.localStorage).filter(function(e3) {
      return e3.startsWith("@@auth0spajs@@");
    });
  }, e2;
}();
var va = function() {
  var e2;
  this.enclosedCache = (e2 = {}, { set: function(t2, n2) {
    e2[t2] = n2;
  }, get: function(t2) {
    var n2 = e2[t2];
    if (n2)
      return n2;
  }, remove: function(t2) {
    delete e2[t2];
  }, allKeys: function() {
    return Object.keys(e2);
  } });
};
var ma = function() {
  function e2(e3, t2, n2) {
    this.cache = e3, this.keyManifest = t2, this.nowProvider = n2, this.nowProvider = this.nowProvider || Uc;
  }
  return e2.prototype.get = function(e3, t2) {
    var n2;
    return void 0 === t2 && (t2 = 0), o(this, void 0, void 0, function() {
      var r2, o2, c2, a2, s2;
      return i(this, function(i2) {
        switch (i2.label) {
          case 0:
            return [4, this.cache.get(e3.toKey())];
          case 1:
            return (r2 = i2.sent()) ? [3, 4] : [4, this.getCacheKeys()];
          case 2:
            return (o2 = i2.sent()) ? (c2 = this.matchExistingCacheKey(e3, o2)) ? [4, this.cache.get(c2)] : [3, 4] : [2];
          case 3:
            r2 = i2.sent(), i2.label = 4;
          case 4:
            return r2 ? [4, this.nowProvider()] : [2];
          case 5:
            return a2 = i2.sent(), s2 = Math.floor(a2 / 1e3), r2.expiresAt - t2 < s2 ? r2.body.refresh_token ? (r2.body = { refresh_token: r2.body.refresh_token }, [4, this.cache.set(e3.toKey(), r2)]) : [3, 7] : [3, 10];
          case 6:
            return i2.sent(), [2, r2.body];
          case 7:
            return [4, this.cache.remove(e3.toKey())];
          case 8:
            return i2.sent(), [4, null === (n2 = this.keyManifest) || void 0 === n2 ? void 0 : n2.remove(e3.toKey())];
          case 9:
            return i2.sent(), [2];
          case 10:
            return [2, r2.body];
        }
      });
    });
  }, e2.prototype.set = function(e3) {
    var t2;
    return o(this, void 0, void 0, function() {
      var n2, r2;
      return i(this, function(o2) {
        switch (o2.label) {
          case 0:
            return n2 = new pa({ client_id: e3.client_id, scope: e3.scope, audience: e3.audience }), [4, this.wrapCacheEntry(e3)];
          case 1:
            return r2 = o2.sent(), [4, this.cache.set(n2.toKey(), r2)];
          case 2:
            return o2.sent(), [4, null === (t2 = this.keyManifest) || void 0 === t2 ? void 0 : t2.add(n2.toKey())];
          case 3:
            return o2.sent(), [2];
        }
      });
    });
  }, e2.prototype.clear = function(e3) {
    var t2;
    return o(this, void 0, void 0, function() {
      var n2, r2 = this;
      return i(this, function(c2) {
        switch (c2.label) {
          case 0:
            return [4, this.getCacheKeys()];
          case 1:
            return (n2 = c2.sent()) ? [4, n2.filter(function(t3) {
              return !e3 || t3.includes(e3);
            }).reduce(function(e4, t3) {
              return o(r2, void 0, void 0, function() {
                return i(this, function(n3) {
                  switch (n3.label) {
                    case 0:
                      return [4, e4];
                    case 1:
                      return n3.sent(), [4, this.cache.remove(t3)];
                    case 2:
                      return n3.sent(), [2];
                  }
                });
              });
            }, Promise.resolve())] : [2];
          case 2:
            return c2.sent(), [4, null === (t2 = this.keyManifest) || void 0 === t2 ? void 0 : t2.clear()];
          case 3:
            return c2.sent(), [2];
        }
      });
    });
  }, e2.prototype.clearSync = function(e3) {
    var t2 = this, n2 = this.cache.allKeys();
    n2 && n2.filter(function(t3) {
      return !e3 || t3.includes(e3);
    }).forEach(function(e4) {
      t2.cache.remove(e4);
    });
  }, e2.prototype.wrapCacheEntry = function(e3) {
    return o(this, void 0, void 0, function() {
      var t2, n2, r2;
      return i(this, function(o2) {
        switch (o2.label) {
          case 0:
            return [4, this.nowProvider()];
          case 1:
            return t2 = o2.sent(), n2 = Math.floor(t2 / 1e3) + e3.expires_in, r2 = Math.min(n2, e3.decodedToken.claims.exp), [2, { body: e3, expiresAt: r2 }];
        }
      });
    });
  }, e2.prototype.getCacheKeys = function() {
    var e3;
    return o(this, void 0, void 0, function() {
      var t2;
      return i(this, function(n2) {
        switch (n2.label) {
          case 0:
            return this.keyManifest ? [4, this.keyManifest.get()] : [3, 2];
          case 1:
            return t2 = null === (e3 = n2.sent()) || void 0 === e3 ? void 0 : e3.keys, [3, 4];
          case 2:
            return [4, this.cache.allKeys()];
          case 3:
            t2 = n2.sent(), n2.label = 4;
          case 4:
            return [2, t2];
        }
      });
    });
  }, e2.prototype.matchExistingCacheKey = function(e3, t2) {
    return t2.filter(function(t3) {
      var n2 = pa.fromKey(t3), r2 = new Set(n2.scope && n2.scope.split(" ")), o2 = e3.scope.split(" "), i2 = n2.scope && o2.reduce(function(e4, t4) {
        return e4 && r2.has(t4);
      }, true);
      return "@@auth0spajs@@" === n2.prefix && n2.client_id === e3.client_id && n2.audience === e3.audience && i2;
    })[0];
  }, e2;
}();
var ba = function() {
  function e2(e3, t2) {
    this.storage = e3, this.clientId = t2, this.storageKey = "".concat("a0.spajs.txs", ".").concat(this.clientId), this.transaction = this.storage.get(this.storageKey);
  }
  return e2.prototype.create = function(e3) {
    this.transaction = e3, this.storage.save(this.storageKey, e3, { daysUntilExpire: 1 });
  }, e2.prototype.get = function() {
    return this.transaction;
  }, e2.prototype.remove = function() {
    delete this.transaction, this.storage.remove(this.storageKey);
  }, e2;
}();
var ga = function(e2) {
  return "number" == typeof e2;
};
var wa = ["iss", "aud", "exp", "nbf", "iat", "jti", "azp", "nonce", "auth_time", "at_hash", "c_hash", "acr", "amr", "sub_jwk", "cnf", "sip_from_tag", "sip_date", "sip_callid", "sip_cseq_num", "sip_via_branch", "orig", "dest", "mky", "events", "toe", "txn", "rph", "sid", "vot", "vtm"];
var Sa = function(e2) {
  if (!e2.id_token)
    throw new Error("ID token is required but missing");
  var t2 = function(e3) {
    var t3 = e3.split("."), n3 = c(t3, 3), r3 = n3[0], o3 = n3[1], i3 = n3[2];
    if (3 !== t3.length || !r3 || !o3 || !i3)
      throw new Error("ID token could not be decoded");
    var a3 = JSON.parse(ra(o3)), s2 = { __raw: e3 }, u2 = {};
    return Object.keys(a3).forEach(function(e4) {
      s2[e4] = a3[e4], wa.includes(e4) || (u2[e4] = a3[e4]);
    }), { encoded: { header: r3, payload: o3, signature: i3 }, header: JSON.parse(ra(r3)), claims: s2, user: u2 };
  }(e2.id_token);
  if (!t2.claims.iss)
    throw new Error("Issuer (iss) claim must be a string present in the ID token");
  if (t2.claims.iss !== e2.iss)
    throw new Error('Issuer (iss) claim mismatch in the ID token; expected "'.concat(e2.iss, '", found "').concat(t2.claims.iss, '"'));
  if (!t2.user.sub)
    throw new Error("Subject (sub) claim must be a string present in the ID token");
  if ("RS256" !== t2.header.alg)
    throw new Error('Signature algorithm of "'.concat(t2.header.alg, '" is not supported. Expected the ID token to be signed with "RS256".'));
  if (!t2.claims.aud || "string" != typeof t2.claims.aud && !Array.isArray(t2.claims.aud))
    throw new Error("Audience (aud) claim must be a string or array of strings present in the ID token");
  if (Array.isArray(t2.claims.aud)) {
    if (!t2.claims.aud.includes(e2.aud))
      throw new Error('Audience (aud) claim mismatch in the ID token; expected "'.concat(e2.aud, '" but was not one of "').concat(t2.claims.aud.join(", "), '"'));
    if (t2.claims.aud.length > 1) {
      if (!t2.claims.azp)
        throw new Error("Authorized Party (azp) claim must be a string present in the ID token when Audience (aud) claim has multiple values");
      if (t2.claims.azp !== e2.aud)
        throw new Error('Authorized Party (azp) claim mismatch in the ID token; expected "'.concat(e2.aud, '", found "').concat(t2.claims.azp, '"'));
    }
  } else if (t2.claims.aud !== e2.aud)
    throw new Error('Audience (aud) claim mismatch in the ID token; expected "'.concat(e2.aud, '" but found "').concat(t2.claims.aud, '"'));
  if (e2.nonce) {
    if (!t2.claims.nonce)
      throw new Error("Nonce (nonce) claim must be a string present in the ID token");
    if (t2.claims.nonce !== e2.nonce)
      throw new Error('Nonce (nonce) claim mismatch in the ID token; expected "'.concat(e2.nonce, '", found "').concat(t2.claims.nonce, '"'));
  }
  if (e2.max_age && !ga(t2.claims.auth_time))
    throw new Error("Authentication Time (auth_time) claim must be a number present in the ID token when Max Age (max_age) is specified");
  if (!ga(t2.claims.exp))
    throw new Error("Expiration Time (exp) claim must be a number present in the ID token");
  if (!ga(t2.claims.iat))
    throw new Error("Issued At (iat) claim must be a number present in the ID token");
  var n2 = e2.leeway || 60, r2 = new Date(e2.now || Date.now()), o2 = /* @__PURE__ */ new Date(0), i2 = /* @__PURE__ */ new Date(0), a2 = /* @__PURE__ */ new Date(0);
  if (a2.setUTCSeconds(parseInt(t2.claims.auth_time) + e2.max_age + n2), o2.setUTCSeconds(t2.claims.exp + n2), i2.setUTCSeconds(t2.claims.nbf - n2), r2 > o2)
    throw new Error("Expiration Time (exp) claim error in the ID token; current time (".concat(r2, ") is after expiration time (").concat(o2, ")"));
  if (ga(t2.claims.nbf) && r2 < i2)
    throw new Error("Not Before time (nbf) claim in the ID token indicates that this token can't be used just yet. Currrent time (".concat(r2, ") is before ").concat(i2));
  if (ga(t2.claims.auth_time) && r2 > a2)
    throw new Error("Authentication Time (auth_time) claim in the ID token indicates that too much time has passed since the last end-user authentication. Currrent time (".concat(r2, ") is after last auth at ").concat(a2));
  if (e2.organizationId) {
    if (!t2.claims.org_id)
      throw new Error("Organization ID (org_id) claim must be a string present in the ID token");
    if (e2.organizationId !== t2.claims.org_id)
      throw new Error('Organization ID (org_id) claim mismatch in the ID token; expected "'.concat(e2.organizationId, '", found "').concat(t2.claims.org_id, '"'));
  }
  return t2;
};
var ka = l(function(e2, t2) {
  var n2 = s && s.__assign || function() {
    return n2 = Object.assign || function(e3) {
      for (var t3, n3 = 1, r3 = arguments.length; n3 < r3; n3++)
        for (var o3 in t3 = arguments[n3])
          Object.prototype.hasOwnProperty.call(t3, o3) && (e3[o3] = t3[o3]);
      return e3;
    }, n2.apply(this, arguments);
  };
  function r2(e3, t3) {
    if (!t3)
      return "";
    var n3 = "; " + e3;
    return true === t3 ? n3 : n3 + "=" + t3;
  }
  function o2(e3, t3, n3) {
    return encodeURIComponent(e3).replace(/%(23|24|26|2B|5E|60|7C)/g, decodeURIComponent).replace(/\(/g, "%28").replace(/\)/g, "%29") + "=" + encodeURIComponent(t3).replace(/%(23|24|26|2B|3A|3C|3E|3D|2F|3F|40|5B|5D|5E|60|7B|7D|7C)/g, decodeURIComponent) + function(e4) {
      if ("number" == typeof e4.expires) {
        var t4 = /* @__PURE__ */ new Date();
        t4.setMilliseconds(t4.getMilliseconds() + 864e5 * e4.expires), e4.expires = t4;
      }
      return r2("Expires", e4.expires ? e4.expires.toUTCString() : "") + r2("Domain", e4.domain) + r2("Path", e4.path) + r2("Secure", e4.secure) + r2("SameSite", e4.sameSite);
    }(n3);
  }
  function i2(e3) {
    for (var t3 = {}, n3 = e3 ? e3.split("; ") : [], r3 = /(%[\dA-F]{2})+/gi, o3 = 0; o3 < n3.length; o3++) {
      var i3 = n3[o3].split("="), c3 = i3.slice(1).join("=");
      '"' === c3.charAt(0) && (c3 = c3.slice(1, -1));
      try {
        t3[i3[0].replace(r3, decodeURIComponent)] = c3.replace(r3, decodeURIComponent);
      } catch (e4) {
      }
    }
    return t3;
  }
  function c2() {
    return i2(document.cookie);
  }
  function a2(e3, t3, r3) {
    document.cookie = o2(e3, t3, n2({ path: "/" }, r3));
  }
  t2.__esModule = true, t2.encode = o2, t2.parse = i2, t2.getAll = c2, t2.get = function(e3) {
    return c2()[e3];
  }, t2.set = a2, t2.remove = function(e3, t3) {
    a2(e3, "", n2(n2({}, t3), { expires: -1 }));
  };
});
u(ka), ka.encode, ka.parse, ka.getAll;
var _a = ka.get;
var Ia = ka.set;
var Oa = ka.remove;
var xa = { get: function(e2) {
  var t2 = _a(e2);
  if (void 0 !== t2)
    return JSON.parse(t2);
}, save: function(e2, t2, n2) {
  var r2 = {};
  "https:" === window.location.protocol && (r2 = { secure: true, sameSite: "none" }), (null == n2 ? void 0 : n2.daysUntilExpire) && (r2.expires = n2.daysUntilExpire), (null == n2 ? void 0 : n2.cookieDomain) && (r2.domain = n2.cookieDomain), Ia(e2, JSON.stringify(t2), r2);
}, remove: function(e2, t2) {
  var n2 = {};
  (null == t2 ? void 0 : t2.cookieDomain) && (n2.domain = t2.cookieDomain), Oa(e2, n2);
} };
var Ta = { get: function(e2) {
  var t2 = xa.get(e2);
  return t2 || xa.get("".concat("_legacy_").concat(e2));
}, save: function(e2, t2, n2) {
  var r2 = {};
  "https:" === window.location.protocol && (r2 = { secure: true }), (null == n2 ? void 0 : n2.daysUntilExpire) && (r2.expires = n2.daysUntilExpire), Ia("".concat("_legacy_").concat(e2), JSON.stringify(t2), r2), xa.save(e2, t2, n2);
}, remove: function(e2, t2) {
  var n2 = {};
  (null == t2 ? void 0 : t2.cookieDomain) && (n2.domain = t2.cookieDomain), Oa(e2, n2), xa.remove(e2, t2), xa.remove("".concat("_legacy_").concat(e2), t2);
} };
var Ca = { get: function(e2) {
  if ("undefined" != typeof sessionStorage) {
    var t2 = sessionStorage.getItem(e2);
    if (void 0 !== t2)
      return JSON.parse(t2);
  }
}, save: function(e2, t2) {
  sessionStorage.setItem(e2, JSON.stringify(t2));
}, remove: function(e2) {
  sessionStorage.removeItem(e2);
} };
function ja(e2, t2, n2) {
  var r2 = void 0 === t2 ? null : t2, o2 = function(e3, t3) {
    var n3 = atob(e3);
    if (t3) {
      for (var r3 = new Uint8Array(n3.length), o3 = 0, i3 = n3.length; o3 < i3; ++o3)
        r3[o3] = n3.charCodeAt(o3);
      return String.fromCharCode.apply(null, new Uint16Array(r3.buffer));
    }
    return n3;
  }(e2, void 0 !== n2 && n2), i2 = o2.indexOf("\n", 10) + 1, c2 = o2.substring(i2) + (r2 ? "//# sourceMappingURL=" + r2 : ""), a2 = new Blob([c2], { type: "application/javascript" });
  return URL.createObjectURL(a2);
}
var La;
var Ra;
var Wa;
var Za;
var Ea = (La = "Lyogcm9sbHVwLXBsdWdpbi13ZWItd29ya2VyLWxvYWRlciAqLwohZnVuY3Rpb24oKXsidXNlIHN0cmljdCI7dmFyIHQ9ZnVuY3Rpb24oZSxyKXtyZXR1cm4gdD1PYmplY3Quc2V0UHJvdG90eXBlT2Z8fHtfX3Byb3RvX186W119aW5zdGFuY2VvZiBBcnJheSYmZnVuY3Rpb24odCxlKXt0Ll9fcHJvdG9fXz1lfXx8ZnVuY3Rpb24odCxlKXtmb3IodmFyIHIgaW4gZSlPYmplY3QucHJvdG90eXBlLmhhc093blByb3BlcnR5LmNhbGwoZSxyKSYmKHRbcl09ZVtyXSl9LHQoZSxyKX07ZnVuY3Rpb24gZShlLHIpe2lmKCJmdW5jdGlvbiIhPXR5cGVvZiByJiZudWxsIT09cil0aHJvdyBuZXcgVHlwZUVycm9yKCJDbGFzcyBleHRlbmRzIHZhbHVlICIrU3RyaW5nKHIpKyIgaXMgbm90IGEgY29uc3RydWN0b3Igb3IgbnVsbCIpO2Z1bmN0aW9uIG4oKXt0aGlzLmNvbnN0cnVjdG9yPWV9dChlLHIpLGUucHJvdG90eXBlPW51bGw9PT1yP09iamVjdC5jcmVhdGUocik6KG4ucHJvdG90eXBlPXIucHJvdG90eXBlLG5ldyBuKX12YXIgcj1mdW5jdGlvbigpe3JldHVybiByPU9iamVjdC5hc3NpZ258fGZ1bmN0aW9uKHQpe2Zvcih2YXIgZSxyPTEsbj1hcmd1bWVudHMubGVuZ3RoO3I8bjtyKyspZm9yKHZhciBvIGluIGU9YXJndW1lbnRzW3JdKU9iamVjdC5wcm90b3R5cGUuaGFzT3duUHJvcGVydHkuY2FsbChlLG8pJiYodFtvXT1lW29dKTtyZXR1cm4gdH0sci5hcHBseSh0aGlzLGFyZ3VtZW50cyl9O2Z1bmN0aW9uIG4odCxlLHIsbil7cmV0dXJuIG5ldyhyfHwocj1Qcm9taXNlKSkoKGZ1bmN0aW9uKG8sYyl7ZnVuY3Rpb24gaSh0KXt0cnl7cyhuLm5leHQodCkpfWNhdGNoKHQpe2ModCl9fWZ1bmN0aW9uIGEodCl7dHJ5e3Mobi50aHJvdyh0KSl9Y2F0Y2godCl7Yyh0KX19ZnVuY3Rpb24gcyh0KXt2YXIgZTt0LmRvbmU/byh0LnZhbHVlKTooZT10LnZhbHVlLGUgaW5zdGFuY2VvZiByP2U6bmV3IHIoKGZ1bmN0aW9uKHQpe3QoZSl9KSkpLnRoZW4oaSxhKX1zKChuPW4uYXBwbHkodCxlfHxbXSkpLm5leHQoKSl9KSl9ZnVuY3Rpb24gbyh0LGUpe3ZhciByLG4sbyxjLGk9e2xhYmVsOjAsc2VudDpmdW5jdGlvbigpe2lmKDEmb1swXSl0aHJvdyBvWzFdO3JldHVybiBvWzFdfSx0cnlzOltdLG9wczpbXX07cmV0dXJuIGM9e25leHQ6YSgwKSx0aHJvdzphKDEpLHJldHVybjphKDIpfSwiZnVuY3Rpb24iPT10eXBlb2YgU3ltYm9sJiYoY1tTeW1ib2wuaXRlcmF0b3JdPWZ1bmN0aW9uKCl7cmV0dXJuIHRoaXN9KSxjO2Z1bmN0aW9uIGEoYyl7cmV0dXJuIGZ1bmN0aW9uKGEpe3JldHVybiBmdW5jdGlvbihjKXtpZihyKXRocm93IG5ldyBUeXBlRXJyb3IoIkdlbmVyYXRvciBpcyBhbHJlYWR5IGV4ZWN1dGluZy4iKTtmb3IoO2k7KXRyeXtpZihyPTEsbiYmKG89MiZjWzBdP24ucmV0dXJuOmNbMF0/bi50aHJvd3x8KChvPW4ucmV0dXJuKSYmby5jYWxsKG4pLDApOm4ubmV4dCkmJiEobz1vLmNhbGwobixjWzFdKSkuZG9uZSlyZXR1cm4gbztzd2l0Y2gobj0wLG8mJihjPVsyJmNbMF0sby52YWx1ZV0pLGNbMF0pe2Nhc2UgMDpjYXNlIDE6bz1jO2JyZWFrO2Nhc2UgNDpyZXR1cm4gaS5sYWJlbCsrLHt2YWx1ZTpjWzFdLGRvbmU6ITF9O2Nhc2UgNTppLmxhYmVsKyssbj1jWzFdLGM9WzBdO2NvbnRpbnVlO2Nhc2UgNzpjPWkub3BzLnBvcCgpLGkudHJ5cy5wb3AoKTtjb250aW51ZTtkZWZhdWx0OmlmKCEobz1pLnRyeXMsKG89by5sZW5ndGg+MCYmb1tvLmxlbmd0aC0xXSl8fDYhPT1jWzBdJiYyIT09Y1swXSkpe2k9MDtjb250aW51ZX1pZigzPT09Y1swXSYmKCFvfHxjWzFdPm9bMF0mJmNbMV08b1szXSkpe2kubGFiZWw9Y1sxXTticmVha31pZig2PT09Y1swXSYmaS5sYWJlbDxvWzFdKXtpLmxhYmVsPW9bMV0sbz1jO2JyZWFrfWlmKG8mJmkubGFiZWw8b1syXSl7aS5sYWJlbD1vWzJdLGkub3BzLnB1c2goYyk7YnJlYWt9b1syXSYmaS5vcHMucG9wKCksaS50cnlzLnBvcCgpO2NvbnRpbnVlfWM9ZS5jYWxsKHQsaSl9Y2F0Y2godCl7Yz1bNix0XSxuPTB9ZmluYWxseXtyPW89MH1pZig1JmNbMF0pdGhyb3cgY1sxXTtyZXR1cm57dmFsdWU6Y1swXT9jWzFdOnZvaWQgMCxkb25lOiEwfX0oW2MsYV0pfX19ZnVuY3Rpb24gYyh0LGUpe3JldHVybiB2b2lkIDA9PT1lJiYoZT1bXSksdCYmIWUuaW5jbHVkZXModCk/dDoiIn12YXIgaT1mdW5jdGlvbih0KXtmdW5jdGlvbiByKGUsbil7dmFyIG89dC5jYWxsKHRoaXMsbil8fHRoaXM7cmV0dXJuIG8uZXJyb3I9ZSxvLmVycm9yX2Rlc2NyaXB0aW9uPW4sT2JqZWN0LnNldFByb3RvdHlwZU9mKG8sci5wcm90b3R5cGUpLG99cmV0dXJuIGUocix0KSxyLmZyb21QYXlsb2FkPWZ1bmN0aW9uKHQpe3JldHVybiBuZXcgcih0LmVycm9yLHQuZXJyb3JfZGVzY3JpcHRpb24pfSxyfShFcnJvcik7IWZ1bmN0aW9uKHQpe2Z1bmN0aW9uIHIoZSxuLG8sYyl7dm9pZCAwPT09YyYmKGM9bnVsbCk7dmFyIGk9dC5jYWxsKHRoaXMsZSxuKXx8dGhpcztyZXR1cm4gaS5zdGF0ZT1vLGkuYXBwU3RhdGU9YyxPYmplY3Quc2V0UHJvdG90eXBlT2YoaSxyLnByb3RvdHlwZSksaX1lKHIsdCl9KGkpLGZ1bmN0aW9uKHQpe2Z1bmN0aW9uIHIoZSl7dmFyIG49dC5jYWxsKHRoaXMpfHx0aGlzO3JldHVybiBuLnBvcHVwPWUsT2JqZWN0LnNldFByb3RvdHlwZU9mKG4sci5wcm90b3R5cGUpLG59ZShyLHQpfShmdW5jdGlvbih0KXtmdW5jdGlvbiByKCl7dmFyIGU9dC5jYWxsKHRoaXMsInRpbWVvdXQiLCJUaW1lb3V0Iil8fHRoaXM7cmV0dXJuIE9iamVjdC5zZXRQcm90b3R5cGVPZihlLHIucHJvdG90eXBlKSxlfXJldHVybiBlKHIsdCkscn0oaSkpLGZ1bmN0aW9uKHQpe2Z1bmN0aW9uIHIoZSl7dmFyIG49dC5jYWxsKHRoaXMsImNhbmNlbGxlZCIsIlBvcHVwIGNsb3NlZCIpfHx0aGlzO3JldHVybiBuLnBvcHVwPWUsT2JqZWN0LnNldFByb3RvdHlwZU9mKG4sci5wcm90b3R5cGUpLG59ZShyLHQpfShpKSxmdW5jdGlvbih0KXtmdW5jdGlvbiByKGUsbixvKXt2YXIgYz10LmNhbGwodGhpcyxlLG4pfHx0aGlzO3JldHVybiBjLm1mYV90b2tlbj1vLE9iamVjdC5zZXRQcm90b3R5cGVPZihjLHIucHJvdG90eXBlKSxjfWUocix0KX0oaSk7dmFyIGE9ZnVuY3Rpb24odCl7ZnVuY3Rpb24gcihlLG4pe3ZhciBvPXQuY2FsbCh0aGlzLCJtaXNzaW5nX3JlZnJlc2hfdG9rZW4iLCJNaXNzaW5nIFJlZnJlc2ggVG9rZW4gKGF1ZGllbmNlOiAnIi5jb25jYXQoYyhlLFsiZGVmYXVsdCJdKSwiJywgc2NvcGU6ICciKS5jb25jYXQoYyhuKSwiJykiKSl8fHRoaXM7cmV0dXJuIG8uYXVkaWVuY2U9ZSxvLnNjb3BlPW4sT2JqZWN0LnNldFByb3RvdHlwZU9mKG8sci5wcm90b3R5cGUpLG99cmV0dXJuIGUocix0KSxyfShpKSxzPXt9LHU9ZnVuY3Rpb24odCxlKXtyZXR1cm4iIi5jb25jYXQodCwifCIpLmNvbmNhdChlKX07YWRkRXZlbnRMaXN0ZW5lcigibWVzc2FnZSIsKGZ1bmN0aW9uKHQpe3ZhciBlPXQuZGF0YSxjPWUudGltZW91dCxpPWUuYXV0aCxmPWUuZmV0Y2hVcmwsbD1lLmZldGNoT3B0aW9ucyxwPWUudXNlRm9ybURhdGEsaD1mdW5jdGlvbih0LGUpe3ZhciByPSJmdW5jdGlvbiI9PXR5cGVvZiBTeW1ib2wmJnRbU3ltYm9sLml0ZXJhdG9yXTtpZighcilyZXR1cm4gdDt2YXIgbixvLGM9ci5jYWxsKHQpLGk9W107dHJ5e2Zvcig7KHZvaWQgMD09PWV8fGUtLSA+MCkmJiEobj1jLm5leHQoKSkuZG9uZTspaS5wdXNoKG4udmFsdWUpfWNhdGNoKHQpe289e2Vycm9yOnR9fWZpbmFsbHl7dHJ5e24mJiFuLmRvbmUmJihyPWMucmV0dXJuKSYmci5jYWxsKGMpfWZpbmFsbHl7aWYobyl0aHJvdyBvLmVycm9yfX1yZXR1cm4gaX0odC5wb3J0cywxKVswXTtyZXR1cm4gbih2b2lkIDAsdm9pZCAwLHZvaWQgMCwoZnVuY3Rpb24oKXt2YXIgdCxlLG4seSx2LGIsZCx3LE8sXztyZXR1cm4gbyh0aGlzLChmdW5jdGlvbihvKXtzd2l0Y2goby5sYWJlbCl7Y2FzZSAwOm49KGU9aXx8e30pLmF1ZGllbmNlLHk9ZS5zY29wZSxvLmxhYmVsPTE7Y2FzZSAxOmlmKG8udHJ5cy5wdXNoKFsxLDcsLDhdKSwhKHY9cD8obT1sLmJvZHksaz1uZXcgVVJMU2VhcmNoUGFyYW1zKG0pLFA9e30say5mb3JFYWNoKChmdW5jdGlvbih0LGUpe1BbZV09dH0pKSxQKTpKU09OLnBhcnNlKGwuYm9keSkpLnJlZnJlc2hfdG9rZW4mJiJyZWZyZXNoX3Rva2VuIj09PXYuZ3JhbnRfdHlwZSl7aWYoYj1mdW5jdGlvbih0LGUpe3JldHVybiBzW3UodCxlKV19KG4seSksIWIpdGhyb3cgbmV3IGEobix5KTtsLmJvZHk9cD9uZXcgVVJMU2VhcmNoUGFyYW1zKHIocih7fSx2KSx7cmVmcmVzaF90b2tlbjpifSkpLnRvU3RyaW5nKCk6SlNPTi5zdHJpbmdpZnkocihyKHt9LHYpLHtyZWZyZXNoX3Rva2VuOmJ9KSl9ZD12b2lkIDAsImZ1bmN0aW9uIj09dHlwZW9mIEFib3J0Q29udHJvbGxlciYmKGQ9bmV3IEFib3J0Q29udHJvbGxlcixsLnNpZ25hbD1kLnNpZ25hbCksdz12b2lkIDAsby5sYWJlbD0yO2Nhc2UgMjpyZXR1cm4gby50cnlzLnB1c2goWzIsNCwsNV0pLFs0LFByb21pc2UucmFjZShbKGc9YyxuZXcgUHJvbWlzZSgoZnVuY3Rpb24odCl7cmV0dXJuIHNldFRpbWVvdXQodCxnKX0pKSksZmV0Y2goZixyKHt9LGwpKV0pXTtjYXNlIDM6cmV0dXJuIHc9by5zZW50KCksWzMsNV07Y2FzZSA0OnJldHVybiBPPW8uc2VudCgpLGgucG9zdE1lc3NhZ2Uoe2Vycm9yOk8ubWVzc2FnZX0pLFsyXTtjYXNlIDU6cmV0dXJuIHc/WzQsdy5qc29uKCldOihkJiZkLmFib3J0KCksaC5wb3N0TWVzc2FnZSh7ZXJyb3I6IlRpbWVvdXQgd2hlbiBleGVjdXRpbmcgJ2ZldGNoJyJ9KSxbMl0pO2Nhc2UgNjpyZXR1cm4odD1vLnNlbnQoKSkucmVmcmVzaF90b2tlbj8oZnVuY3Rpb24odCxlLHIpe3NbdShlLHIpXT10fSh0LnJlZnJlc2hfdG9rZW4sbix5KSxkZWxldGUgdC5yZWZyZXNoX3Rva2VuKTpmdW5jdGlvbih0LGUpe2RlbGV0ZSBzW3UodCxlKV19KG4seSksaC5wb3N0TWVzc2FnZSh7b2s6dy5vayxqc29uOnR9KSxbMyw4XTtjYXNlIDc6cmV0dXJuIF89by5zZW50KCksaC5wb3N0TWVzc2FnZSh7b2s6ITEsanNvbjp7ZXJyb3JfZGVzY3JpcHRpb246Xy5tZXNzYWdlfX0pLFszLDhdO2Nhc2UgODpyZXR1cm5bMl19dmFyIGcsbSxrLFB9KSl9KSl9KSl9KCk7Cgo=", Ra = null, Wa = false, function(e2) {
  return Za = Za || ja(La, Ra, Wa), new Worker(Za, e2);
});
var Ga = {};
var Aa = function() {
  function e2(e3, t2) {
    this.cache = e3, this.clientId = t2, this.manifestKey = this.createManifestKeyFrom(this.clientId);
  }
  return e2.prototype.add = function(e3) {
    var t2;
    return o(this, void 0, void 0, function() {
      var n2, r2;
      return i(this, function(o2) {
        switch (o2.label) {
          case 0:
            return r2 = Set.bind, [4, this.cache.get(this.manifestKey)];
          case 1:
            return (n2 = new (r2.apply(Set, [void 0, (null === (t2 = o2.sent()) || void 0 === t2 ? void 0 : t2.keys) || []]))()).add(e3), [4, this.cache.set(this.manifestKey, { keys: a([], c(n2), false) })];
          case 2:
            return o2.sent(), [2];
        }
      });
    });
  }, e2.prototype.remove = function(e3) {
    return o(this, void 0, void 0, function() {
      var t2, n2;
      return i(this, function(r2) {
        switch (r2.label) {
          case 0:
            return [4, this.cache.get(this.manifestKey)];
          case 1:
            return (t2 = r2.sent()) ? ((n2 = new Set(t2.keys)).delete(e3), n2.size > 0 ? [4, this.cache.set(this.manifestKey, { keys: a([], c(n2), false) })] : [3, 3]) : [3, 5];
          case 2:
          case 4:
            return [2, r2.sent()];
          case 3:
            return [4, this.cache.remove(this.manifestKey)];
          case 5:
            return [2];
        }
      });
    });
  }, e2.prototype.get = function() {
    return this.cache.get(this.manifestKey);
  }, e2.prototype.clear = function() {
    return this.cache.remove(this.manifestKey);
  }, e2.prototype.createManifestKeyFrom = function(e3) {
    return "".concat("@@auth0spajs@@", "::").concat(e3);
  }, e2;
}();
var Pa = new Xc();
var Xa = { memory: function() {
  return new va().enclosedCache;
}, localstorage: function() {
  return new ya();
} };
var Fa = function(e2) {
  return Xa[e2];
};
var Ka = function() {
  return !/Trident.*rv:11\.0/.test(navigator.userAgent);
};
var Na = function() {
  function e2(e3) {
    var t2, n2, c2, a2, s2 = this;
    if (this.options = e3, this._releaseLockOnPageHide = function() {
      return o(s2, void 0, void 0, function() {
        return i(this, function(e4) {
          switch (e4.label) {
            case 0:
              return [4, Pa.releaseLock("auth0.lock.getTokenSilently")];
            case 1:
              return e4.sent(), window.removeEventListener("pagehide", this._releaseLockOnPageHide), [2];
          }
        });
      });
    }, "undefined" != typeof window && function() {
      if (!Qc())
        throw new Error("For security reasons, `window.crypto` is required to run `auth0-spa-js`.");
      if (void 0 === qc())
        throw new Error("\n      auth0-spa-js must run on a secure origin. See https://github.com/auth0/auth0-spa-js/blob/master/FAQ.md#why-do-i-get-auth0-spa-js-must-run-on-a-secure-origin for more information.\n    ");
    }(), e3.cache && e3.cacheLocation && console.warn("Both `cache` and `cacheLocation` options have been specified in the Auth0Client configuration; ignoring `cacheLocation` and using `cache`."), e3.cache)
      c2 = e3.cache;
    else {
      if (this.cacheLocation = e3.cacheLocation || "memory", !Fa(this.cacheLocation))
        throw new Error('Invalid cache location "'.concat(this.cacheLocation, '"'));
      c2 = Fa(this.cacheLocation)();
    }
    this.httpTimeoutMs = e3.httpTimeoutInSeconds ? 1e3 * e3.httpTimeoutInSeconds : 1e4, this.cookieStorage = false === e3.legacySameSiteCookie ? xa : Ta, this.orgHintCookieName = (a2 = this.options.client_id, "auth0.".concat(a2, ".organization_hint")), this.isAuthenticatedCookieName = function(e4) {
      return "auth0.".concat(e4, ".is.authenticated");
    }(this.options.client_id), this.sessionCheckExpiryDays = e3.sessionCheckExpiryDays || 1;
    var u2, l2 = e3.useCookiesForTransactions ? this.cookieStorage : Ca;
    this.scope = this.options.scope, this.transactionManager = new ba(l2, this.options.client_id), this.nowProvider = this.options.nowProvider || Uc, this.cacheManager = new ma(c2, c2.allKeys ? null : new Aa(c2, this.options.client_id), this.nowProvider), this.domainUrl = (u2 = this.options.domain, /^https?:\/\//.test(u2) ? u2 : "https://".concat(u2)), this.tokenIssuer = function(e4, t3) {
      return e4 ? e4.startsWith("https://") ? e4 : "https://".concat(e4, "/") : "".concat(t3, "/");
    }(this.options.issuer, this.domainUrl), this.defaultScope = ha("openid", void 0 !== (null === (n2 = null === (t2 = this.options) || void 0 === t2 ? void 0 : t2.advancedOptions) || void 0 === n2 ? void 0 : n2.defaultScope) ? this.options.advancedOptions.defaultScope : "openid profile email"), this.options.useRefreshTokens && (this.scope = ha(this.scope, "offline_access")), "undefined" != typeof window && window.Worker && this.options.useRefreshTokens && "memory" === this.cacheLocation && Ka() && (this.worker = new Ea()), this.customOptions = function(e4) {
      return e4.advancedOptions, e4.audience, e4.auth0Client, e4.authorizeTimeoutInSeconds, e4.cacheLocation, e4.cache, e4.client_id, e4.domain, e4.issuer, e4.leeway, e4.max_age, e4.nowProvider, e4.redirect_uri, e4.scope, e4.useRefreshTokens, e4.useRefreshTokensFallback, e4.useCookiesForTransactions, e4.useFormData, r(e4, ["advancedOptions", "audience", "auth0Client", "authorizeTimeoutInSeconds", "cacheLocation", "cache", "client_id", "domain", "issuer", "leeway", "max_age", "nowProvider", "redirect_uri", "scope", "useRefreshTokens", "useRefreshTokensFallback", "useCookiesForTransactions", "useFormData"]);
    }(e3), this.useRefreshTokensFallback = false !== this.options.useRefreshTokensFallback;
  }
  return e2.prototype._url = function(e3) {
    var t2 = encodeURIComponent(btoa(JSON.stringify(this.options.auth0Client || Nc)));
    return "".concat(this.domainUrl).concat(e3, "&auth0Client=").concat(t2);
  }, e2.prototype._getParams = function(e3, t2, o2, i2, c2) {
    var a2 = this.options;
    a2.useRefreshTokens, a2.useCookiesForTransactions, a2.useFormData, a2.auth0Client, a2.cacheLocation, a2.advancedOptions, a2.detailedResponse, a2.nowProvider, a2.authorizeTimeoutInSeconds, a2.legacySameSiteCookie, a2.sessionCheckExpiryDays, a2.domain, a2.leeway, a2.httpTimeoutInSeconds;
    var s2 = r(a2, ["useRefreshTokens", "useCookiesForTransactions", "useFormData", "auth0Client", "cacheLocation", "advancedOptions", "detailedResponse", "nowProvider", "authorizeTimeoutInSeconds", "legacySameSiteCookie", "sessionCheckExpiryDays", "domain", "leeway", "httpTimeoutInSeconds"]);
    return n(n(n({}, s2), e3), { scope: ha(this.defaultScope, this.scope, e3.scope), response_type: "code", response_mode: "query", state: t2, nonce: o2, redirect_uri: c2 || this.options.redirect_uri, code_challenge: i2, code_challenge_method: "S256" });
  }, e2.prototype._authorizeUrl = function(e3) {
    return this._url("/authorize?".concat(ta(e3)));
  }, e2.prototype._verifyIdToken = function(e3, t2, n2) {
    return o(this, void 0, void 0, function() {
      var r2;
      return i(this, function(o2) {
        switch (o2.label) {
          case 0:
            return [4, this.nowProvider()];
          case 1:
            return r2 = o2.sent(), [2, Sa({ iss: this.tokenIssuer, aud: this.options.client_id, id_token: e3, nonce: t2, organizationId: n2, leeway: this.options.leeway, max_age: this._parseNumber(this.options.max_age), now: r2 })];
        }
      });
    });
  }, e2.prototype._parseNumber = function(e3) {
    return "string" != typeof e3 ? e3 : parseInt(e3, 10) || void 0;
  }, e2.prototype._processOrgIdHint = function(e3) {
    e3 ? this.cookieStorage.save(this.orgHintCookieName, e3, { daysUntilExpire: this.sessionCheckExpiryDays, cookieDomain: this.options.cookieDomain }) : this.cookieStorage.remove(this.orgHintCookieName, { cookieDomain: this.options.cookieDomain });
  }, e2.prototype.buildAuthorizeUrl = function(e3) {
    return void 0 === e3 && (e3 = {}), o(this, void 0, void 0, function() {
      var t2, o2, c2, a2, s2, u2, l2, f2, d2, h2, p2, y2;
      return i(this, function(i2) {
        switch (i2.label) {
          case 0:
            return t2 = e3.redirect_uri, o2 = e3.appState, c2 = r(e3, ["redirect_uri", "appState"]), a2 = ea($c()), s2 = ea($c()), u2 = $c(), [4, na(u2)];
          case 1:
            return l2 = i2.sent(), f2 = oa(l2), d2 = e3.fragment ? "#".concat(e3.fragment) : "", h2 = this._getParams(c2, a2, s2, f2, t2), p2 = this._authorizeUrl(h2), y2 = e3.organization || this.options.organization, this.transactionManager.create(n({ nonce: s2, code_verifier: u2, appState: o2, scope: h2.scope, audience: h2.audience || "default", redirect_uri: h2.redirect_uri, state: a2 }, y2 && { organizationId: y2 })), [2, p2 + d2];
        }
      });
    });
  }, e2.prototype.loginWithPopup = function(e3, t2) {
    return o(this, void 0, void 0, function() {
      var o2, c2, a2, s2, u2, l2, f2, d2, h2, p2, y2, v2, m2;
      return i(this, function(i2) {
        switch (i2.label) {
          case 0:
            if (e3 = e3 || {}, !(t2 = t2 || {}).popup && (t2.popup = function(e4) {
              var t3 = window.screenX + (window.innerWidth - 400) / 2, n2 = window.screenY + (window.innerHeight - 600) / 2;
              return window.open(e4, "auth0:authorize:popup", "left=".concat(t3, ",top=").concat(n2, ",width=").concat(400, ",height=").concat(600, ",resizable,scrollbars=yes,status=1"));
            }(""), !t2.popup))
              throw new Error("Unable to open a popup for loginWithPopup - window.open returned `null`");
            return o2 = r(e3, []), c2 = ea($c()), a2 = ea($c()), s2 = $c(), [4, na(s2)];
          case 1:
            return u2 = i2.sent(), l2 = oa(u2), f2 = this._getParams(o2, c2, a2, l2, this.options.redirect_uri || window.location.origin), d2 = this._authorizeUrl(n(n({}, f2), { response_mode: "web_message" })), t2.popup.location.href = d2, [4, Bc(n(n({}, t2), { timeoutInSeconds: t2.timeoutInSeconds || this.options.authorizeTimeoutInSeconds || 60 }))];
          case 2:
            if (h2 = i2.sent(), c2 !== h2.state)
              throw new Error("Invalid state");
            return [4, fa({ audience: f2.audience, scope: f2.scope, baseUrl: this.domainUrl, client_id: this.options.client_id, code_verifier: s2, code: h2.code, grant_type: "authorization_code", redirect_uri: f2.redirect_uri, auth0Client: this.options.auth0Client, useFormData: this.options.useFormData, timeout: this.httpTimeoutMs }, this.worker)];
          case 3:
            return p2 = i2.sent(), y2 = e3.organization || this.options.organization, [4, this._verifyIdToken(p2.id_token, a2, y2)];
          case 4:
            return v2 = i2.sent(), m2 = n(n({}, p2), { decodedToken: v2, scope: f2.scope, audience: f2.audience || "default", client_id: this.options.client_id }), [4, this.cacheManager.set(m2)];
          case 5:
            return i2.sent(), this.cookieStorage.save(this.isAuthenticatedCookieName, true, { daysUntilExpire: this.sessionCheckExpiryDays, cookieDomain: this.options.cookieDomain }), this._processOrgIdHint(v2.claims.org_id), [2];
        }
      });
    });
  }, e2.prototype.getUser = function(e3) {
    return void 0 === e3 && (e3 = {}), o(this, void 0, void 0, function() {
      var t2, n2, r2;
      return i(this, function(o2) {
        switch (o2.label) {
          case 0:
            return t2 = e3.audience || this.options.audience || "default", n2 = ha(this.defaultScope, this.scope, e3.scope), [4, this.cacheManager.get(new pa({ client_id: this.options.client_id, audience: t2, scope: n2 }))];
          case 1:
            return [2, (r2 = o2.sent()) && r2.decodedToken && r2.decodedToken.user];
        }
      });
    });
  }, e2.prototype.getIdTokenClaims = function(e3) {
    return void 0 === e3 && (e3 = {}), o(this, void 0, void 0, function() {
      var t2, n2, r2;
      return i(this, function(o2) {
        switch (o2.label) {
          case 0:
            return t2 = e3.audience || this.options.audience || "default", n2 = ha(this.defaultScope, this.scope, e3.scope), [4, this.cacheManager.get(new pa({ client_id: this.options.client_id, audience: t2, scope: n2 }))];
          case 1:
            return [2, (r2 = o2.sent()) && r2.decodedToken && r2.decodedToken.claims];
        }
      });
    });
  }, e2.prototype.loginWithRedirect = function(e3) {
    return void 0 === e3 && (e3 = {}), o(this, void 0, void 0, function() {
      var t2, n2, o2;
      return i(this, function(i2) {
        switch (i2.label) {
          case 0:
            return t2 = e3.redirectMethod, n2 = r(e3, ["redirectMethod"]), [4, this.buildAuthorizeUrl(n2)];
          case 1:
            return o2 = i2.sent(), window.location[t2 || "assign"](o2), [2];
        }
      });
    });
  }, e2.prototype.handleRedirectCallback = function(e3) {
    return void 0 === e3 && (e3 = window.location.href), o(this, void 0, void 0, function() {
      var t2, r2, o2, a2, s2, u2, l2, f2, d2, h2;
      return i(this, function(i2) {
        switch (i2.label) {
          case 0:
            if (0 === (t2 = e3.split("?").slice(1)).length)
              throw new Error("There are no query params available for parsing.");
            if (r2 = function(e4) {
              e4.indexOf("#") > -1 && (e4 = e4.substr(0, e4.indexOf("#")));
              var t3 = e4.split("&"), n2 = {};
              return t3.forEach(function(e5) {
                var t4 = c(e5.split("="), 2), r3 = t4[0], o3 = t4[1];
                n2[r3] = decodeURIComponent(o3);
              }), n2.expires_in && (n2.expires_in = parseInt(n2.expires_in)), n2;
            }(t2.join("")), o2 = r2.state, a2 = r2.code, s2 = r2.error, u2 = r2.error_description, !(l2 = this.transactionManager.get()))
              throw new Error("Invalid state");
            if (this.transactionManager.remove(), s2)
              throw new Hc(s2, u2, o2, l2.appState);
            if (!l2.code_verifier || l2.state && l2.state !== o2)
              throw new Error("Invalid state");
            return f2 = { audience: l2.audience, scope: l2.scope, baseUrl: this.domainUrl, client_id: this.options.client_id, code_verifier: l2.code_verifier, grant_type: "authorization_code", code: a2, auth0Client: this.options.auth0Client, useFormData: this.options.useFormData, timeout: this.httpTimeoutMs }, void 0 !== l2.redirect_uri && (f2.redirect_uri = l2.redirect_uri), [4, fa(f2, this.worker)];
          case 1:
            return d2 = i2.sent(), [4, this._verifyIdToken(d2.id_token, l2.nonce, l2.organizationId)];
          case 2:
            return h2 = i2.sent(), [4, this.cacheManager.set(n(n(n(n({}, d2), { decodedToken: h2, audience: l2.audience, scope: l2.scope }), d2.scope ? { oauthTokenScope: d2.scope } : null), { client_id: this.options.client_id }))];
          case 3:
            return i2.sent(), this.cookieStorage.save(this.isAuthenticatedCookieName, true, { daysUntilExpire: this.sessionCheckExpiryDays, cookieDomain: this.options.cookieDomain }), this._processOrgIdHint(h2.claims.org_id), [2, { appState: l2.appState }];
        }
      });
    });
  }, e2.prototype.checkSession = function(e3) {
    return o(this, void 0, void 0, function() {
      var t2;
      return i(this, function(n2) {
        switch (n2.label) {
          case 0:
            if (!this.cookieStorage.get(this.isAuthenticatedCookieName)) {
              if (!this.cookieStorage.get("auth0.is.authenticated"))
                return [2];
              this.cookieStorage.save(this.isAuthenticatedCookieName, true, { daysUntilExpire: this.sessionCheckExpiryDays, cookieDomain: this.options.cookieDomain }), this.cookieStorage.remove("auth0.is.authenticated");
            }
            n2.label = 1;
          case 1:
            return n2.trys.push([1, 3, , 4]), [4, this.getTokenSilently(e3)];
          case 2:
            return n2.sent(), [3, 4];
          case 3:
            if (t2 = n2.sent(), !Kc.includes(t2.error))
              throw t2;
            return [3, 4];
          case 4:
            return [2];
        }
      });
    });
  }, e2.prototype.getTokenSilently = function(e3) {
    return void 0 === e3 && (e3 = {}), o(this, void 0, void 0, function() {
      var t2, o2, c2, a2, s2 = this;
      return i(this, function(i2) {
        switch (i2.label) {
          case 0:
            return t2 = n(n({ audience: this.options.audience, ignoreCache: false }, e3), { scope: ha(this.defaultScope, this.scope, e3.scope) }), o2 = t2.ignoreCache, c2 = r(t2, ["ignoreCache"]), [4, (u2 = function() {
              return s2._getTokenSilently(n({ ignoreCache: o2 }, c2));
            }, l2 = "".concat(this.options.client_id, "::").concat(c2.audience, "::").concat(c2.scope), f2 = Ga[l2], f2 || (f2 = u2().finally(function() {
              delete Ga[l2], f2 = null;
            }), Ga[l2] = f2), f2)];
          case 1:
            return a2 = i2.sent(), [2, e3.detailedResponse ? a2 : a2.access_token];
        }
        var u2, l2, f2;
      });
    });
  }, e2.prototype._getTokenSilently = function(e3) {
    return void 0 === e3 && (e3 = {}), o(this, void 0, void 0, function() {
      var t2, c2, a2, s2, u2, l2, f2, d2, h2;
      return i(this, function(p2) {
        switch (p2.label) {
          case 0:
            return t2 = e3.ignoreCache, c2 = r(e3, ["ignoreCache"]), t2 ? [3, 2] : [4, this._getEntryFromCache({ scope: c2.scope, audience: c2.audience || "default", client_id: this.options.client_id })];
          case 1:
            if (a2 = p2.sent())
              return [2, a2];
            p2.label = 2;
          case 2:
            return [4, (y2 = function() {
              return Pa.acquireLock("auth0.lock.getTokenSilently", 5e3);
            }, v2 = 10, void 0 === v2 && (v2 = 3), o(void 0, void 0, void 0, function() {
              var e4;
              return i(this, function(t3) {
                switch (t3.label) {
                  case 0:
                    e4 = 0, t3.label = 1;
                  case 1:
                    return e4 < v2 ? [4, y2()] : [3, 4];
                  case 2:
                    if (t3.sent())
                      return [2, true];
                    t3.label = 3;
                  case 3:
                    return e4++, [3, 1];
                  case 4:
                    return [2, false];
                }
              });
            }))];
          case 3:
            if (!p2.sent())
              return [3, 15];
            p2.label = 4;
          case 4:
            return p2.trys.push([4, , 12, 14]), window.addEventListener("pagehide", this._releaseLockOnPageHide), t2 ? [3, 6] : [4, this._getEntryFromCache({ scope: c2.scope, audience: c2.audience || "default", client_id: this.options.client_id })];
          case 5:
            if (a2 = p2.sent())
              return [2, a2];
            p2.label = 6;
          case 6:
            return this.options.useRefreshTokens ? [4, this._getTokenUsingRefreshToken(c2)] : [3, 8];
          case 7:
            return u2 = p2.sent(), [3, 10];
          case 8:
            return [4, this._getTokenFromIFrame(c2)];
          case 9:
            u2 = p2.sent(), p2.label = 10;
          case 10:
            return s2 = u2, [4, this.cacheManager.set(n({ client_id: this.options.client_id }, s2))];
          case 11:
            return p2.sent(), this.cookieStorage.save(this.isAuthenticatedCookieName, true, { daysUntilExpire: this.sessionCheckExpiryDays, cookieDomain: this.options.cookieDomain }), l2 = s2.id_token, f2 = s2.access_token, d2 = s2.oauthTokenScope, h2 = s2.expires_in, [2, n(n({ id_token: l2, access_token: f2 }, d2 ? { scope: d2 } : null), { expires_in: h2 })];
          case 12:
            return [4, Pa.releaseLock("auth0.lock.getTokenSilently")];
          case 13:
            return p2.sent(), window.removeEventListener("pagehide", this._releaseLockOnPageHide), [7];
          case 14:
            return [3, 16];
          case 15:
            throw new Yc();
          case 16:
            return [2];
        }
        var y2, v2;
      });
    });
  }, e2.prototype.getTokenWithPopup = function(e3, t2) {
    return void 0 === e3 && (e3 = {}), void 0 === t2 && (t2 = {}), o(this, void 0, void 0, function() {
      return i(this, function(r2) {
        switch (r2.label) {
          case 0:
            return e3.audience = e3.audience || this.options.audience, e3.scope = ha(this.defaultScope, this.scope, e3.scope), t2 = n(n({}, Fc), t2), [4, this.loginWithPopup(e3, t2)];
          case 1:
            return r2.sent(), [4, this.cacheManager.get(new pa({ scope: e3.scope, audience: e3.audience || "default", client_id: this.options.client_id }))];
          case 2:
            return [2, r2.sent().access_token];
        }
      });
    });
  }, e2.prototype.isAuthenticated = function() {
    return o(this, void 0, void 0, function() {
      return i(this, function(e3) {
        switch (e3.label) {
          case 0:
            return [4, this.getUser()];
          case 1:
            return [2, !!e3.sent()];
        }
      });
    });
  }, e2.prototype.buildLogoutUrl = function(e3) {
    void 0 === e3 && (e3 = {}), null !== e3.client_id ? e3.client_id = e3.client_id || this.options.client_id : delete e3.client_id;
    var t2 = e3.federated, n2 = r(e3, ["federated"]), o2 = t2 ? "&federated" : "";
    return this._url("/v2/logout?".concat(ta(n2))) + o2;
  }, e2.prototype.logout = function(e3) {
    var t2 = this;
    void 0 === e3 && (e3 = {});
    var n2 = e3.localOnly, o2 = r(e3, ["localOnly"]);
    if (n2 && o2.federated)
      throw new Error("It is invalid to set both the `federated` and `localOnly` options to `true`");
    var i2 = function() {
      if (t2.cookieStorage.remove(t2.orgHintCookieName, { cookieDomain: t2.options.cookieDomain }), t2.cookieStorage.remove(t2.isAuthenticatedCookieName, { cookieDomain: t2.options.cookieDomain }), !n2) {
        var e4 = t2.buildLogoutUrl(o2);
        window.location.assign(e4);
      }
    };
    if (this.options.cache)
      return this.cacheManager.clear().then(function() {
        return i2();
      });
    this.cacheManager.clearSync(), i2();
  }, e2.prototype._getTokenFromIFrame = function(e3) {
    return o(this, void 0, void 0, function() {
      var t2, o2, c2, a2, s2, u2, l2, f2, d2, h2, p2, y2, v2, m2, b2, g2, w2;
      return i(this, function(i2) {
        switch (i2.label) {
          case 0:
            return t2 = ea($c()), o2 = ea($c()), c2 = $c(), [4, na(c2)];
          case 1:
            a2 = i2.sent(), s2 = oa(a2), u2 = r(e3, ["detailedResponse"]), l2 = this._getParams(u2, t2, o2, s2, e3.redirect_uri || this.options.redirect_uri || window.location.origin), (f2 = this.cookieStorage.get(this.orgHintCookieName)) && !l2.organization && (l2.organization = f2), d2 = this._authorizeUrl(n(n({}, l2), { prompt: "none", response_mode: "web_message" })), i2.label = 2;
          case 2:
            if (i2.trys.push([2, 6, , 7]), window.crossOriginIsolated)
              throw new Dc("login_required", "The application is running in a Cross-Origin Isolated context, silently retrieving a token without refresh token is not possible.");
            return h2 = e3.timeoutInSeconds || this.options.authorizeTimeoutInSeconds, [4, (S2 = d2, k2 = this.domainUrl, _2 = h2, void 0 === _2 && (_2 = 60), new Promise(function(e4, t3) {
              var n2 = window.document.createElement("iframe");
              n2.setAttribute("width", "0"), n2.setAttribute("height", "0"), n2.style.display = "none";
              var r2, o3 = function() {
                window.document.body.contains(n2) && (window.document.body.removeChild(n2), window.removeEventListener("message", r2, false));
              }, i3 = setTimeout(function() {
                t3(new Yc()), o3();
              }, 1e3 * _2);
              r2 = function(n3) {
                if (n3.origin == k2 && n3.data && "authorization_response" === n3.data.type) {
                  var c3 = n3.source;
                  c3 && c3.close(), n3.data.response.error ? t3(Dc.fromPayload(n3.data.response)) : e4(n3.data.response), clearTimeout(i3), window.removeEventListener("message", r2, false), setTimeout(o3, 2e3);
                }
              }, window.addEventListener("message", r2, false), window.document.body.appendChild(n2), n2.setAttribute("src", S2);
            }))];
          case 3:
            if (p2 = i2.sent(), t2 !== p2.state)
              throw new Error("Invalid state");
            return y2 = e3.scope, v2 = e3.audience, m2 = r(e3, ["scope", "audience", "redirect_uri", "ignoreCache", "timeoutInSeconds", "detailedResponse"]), [4, fa(n(n(n({}, this.customOptions), m2), { scope: y2, audience: v2, baseUrl: this.domainUrl, client_id: this.options.client_id, code_verifier: c2, code: p2.code, grant_type: "authorization_code", redirect_uri: l2.redirect_uri, auth0Client: this.options.auth0Client, useFormData: this.options.useFormData, timeout: m2.timeout || this.httpTimeoutMs }), this.worker)];
          case 4:
            return b2 = i2.sent(), [4, this._verifyIdToken(b2.id_token, o2)];
          case 5:
            return g2 = i2.sent(), this._processOrgIdHint(g2.claims.org_id), [2, n(n({}, b2), { decodedToken: g2, scope: l2.scope, oauthTokenScope: b2.scope, audience: l2.audience || "default" })];
          case 6:
            throw "login_required" === (w2 = i2.sent()).error && this.logout({ localOnly: true }), w2;
          case 7:
            return [2];
        }
        var S2, k2, _2;
      });
    });
  }, e2.prototype._getTokenUsingRefreshToken = function(e3) {
    return o(this, void 0, void 0, function() {
      var t2, o2, c2, a2, s2, u2, l2, f2, d2;
      return i(this, function(i2) {
        switch (i2.label) {
          case 0:
            return e3.scope = ha(this.defaultScope, this.options.scope, e3.scope), [4, this.cacheManager.get(new pa({ scope: e3.scope, audience: e3.audience || "default", client_id: this.options.client_id }))];
          case 1:
            return (t2 = i2.sent()) && t2.refresh_token || this.worker ? [3, 4] : this.useRefreshTokensFallback ? [4, this._getTokenFromIFrame(e3)] : [3, 3];
          case 2:
            return [2, i2.sent()];
          case 3:
            throw new Mc(e3.audience || "default", e3.scope);
          case 4:
            o2 = e3.redirect_uri || this.options.redirect_uri || window.location.origin, a2 = e3.scope, s2 = e3.audience, u2 = r(e3, ["scope", "audience", "ignoreCache", "timeoutInSeconds", "detailedResponse"]), l2 = "number" == typeof e3.timeoutInSeconds ? 1e3 * e3.timeoutInSeconds : null, i2.label = 5;
          case 5:
            return i2.trys.push([5, 7, , 10]), [4, fa(n(n(n(n(n({}, this.customOptions), u2), { audience: s2, scope: a2, baseUrl: this.domainUrl, client_id: this.options.client_id, grant_type: "refresh_token", refresh_token: t2 && t2.refresh_token, redirect_uri: o2 }), l2 && { timeout: l2 }), { auth0Client: this.options.auth0Client, useFormData: this.options.useFormData, timeout: this.httpTimeoutMs }), this.worker)];
          case 6:
            return c2 = i2.sent(), [3, 10];
          case 7:
            return ((f2 = i2.sent()).message.indexOf("Missing Refresh Token") > -1 || f2.message && f2.message.indexOf("invalid refresh token") > -1) && this.useRefreshTokensFallback ? [4, this._getTokenFromIFrame(e3)] : [3, 9];
          case 8:
            return [2, i2.sent()];
          case 9:
            throw f2;
          case 10:
            return [4, this._verifyIdToken(c2.id_token)];
          case 11:
            return d2 = i2.sent(), [2, n(n({}, c2), { decodedToken: d2, scope: e3.scope, oauthTokenScope: c2.scope, audience: e3.audience || "default" })];
        }
      });
    });
  }, e2.prototype._getEntryFromCache = function(e3) {
    var t2 = e3.scope, r2 = e3.audience, c2 = e3.client_id;
    return o(this, void 0, void 0, function() {
      var e4, o2, a2, s2, u2;
      return i(this, function(i2) {
        switch (i2.label) {
          case 0:
            return [4, this.cacheManager.get(new pa({ scope: t2, audience: r2, client_id: c2 }), 60)];
          case 1:
            return (e4 = i2.sent()) && e4.access_token ? (o2 = e4.id_token, a2 = e4.access_token, s2 = e4.oauthTokenScope, u2 = e4.expires_in, [2, n(n({ id_token: o2, access_token: a2 }, s2 ? { scope: s2 } : null), { expires_in: u2 })]) : [2];
        }
      });
    });
  }, e2;
}();
var Ua = function() {
};
function Da(e2) {
  return o(this, void 0, void 0, function() {
    var t2;
    return i(this, function(n2) {
      switch (n2.label) {
        case 0:
          return [4, (t2 = new Na(e2)).checkSession()];
        case 1:
          return n2.sent(), [2, t2];
      }
    });
  });
}
export {
  Na as Auth0Client,
  Hc as AuthenticationError,
  Dc as GenericError,
  va as InMemoryCache,
  ya as LocalStorageCache,
  zc as MfaRequiredError,
  Vc as PopupCancelledError,
  Jc as PopupTimeoutError,
  Yc as TimeoutError,
  Ua as User,
  Da as default
};
//# sourceMappingURL=@auth0_auth0-spa-js.js.map
