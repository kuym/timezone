const geom = require("./geom");

function Quantize(latLongPoints) {
  const points = latLongPoints.map(function(v) {
    //return [parseInt(32768 * v[0] / 180), parseInt(16384 * v[1] / 90)];
    // longitude is 20-bit (sign + 19 bits resolution); latitude is 19-bit (sign + 18 bits resolution)
    return [parseInt(524288 * v[0] / 180), parseInt(262144 * v[1] / 90)];
  });

  // Return deduplicated points
  let last = points[0];
  return points.filter(function(v, i){
    const isDifferent = ((i == 0) || !geom.VIsZero(geom.VSub(v, last)));
    last = v;
    return isDifferent;
  });
}

// Ramer–Douglas–Peucker polygon simplification algorithm
// https://en.wikipedia.org/wiki/Ramer–Douglas–Peucker_algorithm
function SimplifyRDP(points, epsilon) {
  function simplifyRDPInternal(points, epsilon, start, end) {
    if(start < 0 || start >= points.length || end < 0 || end >= points.length || start >= end) {
      return;
    }

    let furthest = -1;
    let maxDist = 0;
    const ab = geom.VSub(points[end], points[start]);
    for(let i = start + 1; i < end; i++) {
      const dist = geom.VDistToLine(points[start], points[end], points[i]);
      if(dist > maxDist) {
        maxDist = dist;
        furthest = i;
      }
    }

    if((furthest >= 0) && (maxDist > epsilon)) {
      // recurse to second half first, as it will modify the `points` array
      simplifyRDPInternal(points, epsilon, furthest, end);
      simplifyRDPInternal(points, epsilon, start, furthest);
    } else {
      // cut all points between `start` and `end`, exclusive
      points.splice(start + 1, end - start - 1);
    }
  }

  const p = points.slice();
  const midpoint = (p.length & ~1) / 2;
  simplifyRDPInternal(p, epsilon, midpoint, p.length - 1);
  simplifyRDPInternal(p, epsilon, 0, midpoint);
  return p;
}

TimeZone.prototype = {
  polygons: undefined,

  BytePack: function BytePack() {
    for(let i = 0; i < this.polygons.length; i++) {
      ;
    }
  }
};
function TimeZone() {
  ;
}

module.exports = {
  Quantize: Quantize,
  SimplifyRDP: SimplifyRDP,
};


/*crunch: function crunch(ci)
{
  let crunchCount = 0;
  const co = [];
  for(let i = 0; i < ci.length; i++)
  {
    if((ci[i][0] == 0) && (ci[i][1] == 0))
      continue;

    const v = ci[i], align = ((co.length > 0)? geom.VDot(co[co.length - 1], v) / (geom.VMag(co[co.length - 1]) * geom.VMag(v)) : 0);
    if((align >= 0.9999) || (align <= -0.999))
    {
      crunchCount++;
    }
    else if(crunchCount > 1)
    {
      vCrunch = co[co.length - crunchCount];
      for(let cc = 1; cc < crunchCount; cc++)
      {
        vCrunch = geom.VAdd(vCrunch, co[co.length - crunchCount + cc]);
      }

      // assert that the crunched segment is identical
      const np = trace(co.slice(co.length - crunchCount, co.length), [0, 0]);
      if((vCrunch[0] != np[0]) || (vCrunch[1] != np[1]))
        throw new Error("Assertion failed: vCrunch[]([" + vCrunch.join(", ") + "]) != np[]([" + np.join(", ") + "])");

      co.splice(co.length - crunchCount, crunchCount, vCrunch);
      crunchCount = 0;
    }
    co.push(v);
  }
  return(co);
}

function crunchOptimized(co)
{
  let crunchCount = 0;
  for(let i = 0; i < co.length; i++)
  {
    if((co[i][0] == 0) && (co[i][1] == 0))
    {
      co.splice(i, 1);
      i -= 1;
      continue;
    }

    const align = (i > 0)? (geom.VDot(co[i - 1 - crunchCount], co[i]) / (geom.VMag(co[i - 1 - crunchCount]) * geom.VMag(co[i]))) : 0;
    if((align >= 0.9999) || (align <= -0.999))
    {
      crunchCount++;
    }
    else if(crunchCount > 0)
    {
      vCrunch = co[i - 1 - crunchCount];
      for(let cc = 0; cc < crunchCount; cc++)
        vCrunch = geom.VAdd(vCrunch, co[i - crunchCount + cc]);

      co.splice(i - 1 - crunchCount, crunchCount + 1, vCrunch);

      i -= (crunchCount + 2);
      crunchCount = 0;
    }
  }
}
*/


function normalizeForBytePack(co)
{
  for(let i = 0; i < co.length; i++)
  {
    const dx = co[i][0], dy = co[i][1];
    let min = 0, max = 0;
    if(dx < min) min = dx;
    if(dy < min) min = dy;
    if(dx > max) max = dx;
    if(dy > max) min = dy;
    if(min < -128 || max > 127)
    {
      console.log("would add about", parseInt(((-min > max)? -min : max) / 127), "more vectors:", [dx, dy]);
    }
  }
}


function old() {

  const tzgeom = 
    //require("/Users/kuy/Downloads/dist-2/Africa@Abidjan.json")
    require("/Users/kuy/Downloads/dist-2/America@Los_Angeles.json");
    // if(type == "FeatureCollection") features[0]
  const ci = tzgeom.geometry.coordinates[0].map(v => [parseInt(32768 * v[0] / 180), parseInt(16384 * v[1] / 90)]);


  const epsilon = 5;
  const midpoint = (ci.length & ~1) / 2;
  simplifyRDP(ci, 5, midpoint, ci.length - 1);
  simplifyRDP(ci, 5, 0, midpoint);

  process.stdout.write(ci.map(v => "[" + v[0] + "," + v[1] + "]").join(", ") + "\n");

  // turn ci (a set of points) into a vector loop
  let co = [];
  for(let i = 0; i < ci.length; i++)
  {
    const v = geom.VSub(ci[(i == (ci.length - 1))? 0 : (i + 1)], ci[i]);
    if((v[0] == 0) && (v[1] == 0))
      continue;
    co.push(v);
  }
  crunchOptimized(co);

  let ccount = 0;
  const pend = ci[ci.length - 1];
  const p = trace(co, ci[0]);
  if((p[0] != pend[0]) || (p[1] != pend[1]))
    console.log("Assertion failed: co[](", p, ") != ci[](", pend, ")");
  while(true)
  {
    const cn = crunch(co);
    console.log("crunch #", ++ccount, co.length, cn.length);

    const p = trace(cn, ci[0]);
    if((p[0] != pend[0]) || (p[1] != pend[1]))
    {
      console.log("Assertion failed: co[](", p, ") != ci[](", pend, ")");
      break;
    }

    if(co.length == cn.length)
      break;
    co = cn;
  }

  normalizeForBytePack(co);

  console.log("in/out:", ci.length, co.length);
  console.log("origin=", ci[0], "aabb=", polyAABB(ci));
  process.stdout.write(co.map(v => "[" + v[0] + "," + v[1] + "]").join(", ") + "\n");
}
