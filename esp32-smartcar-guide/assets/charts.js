// assets/charts.js —— PID 调参示意曲线（图 4）
(function() {
  var style = getComputedStyle(document.documentElement);
  var accent = style.getPropertyValue('--accent').trim();
  var accent2 = style.getPropertyValue('--accent2').trim();
  var ink = style.getPropertyValue('--ink').trim();
  var muted = style.getPropertyValue('--muted').trim();
  var rule = style.getPropertyValue('--rule').trim();

  // 模拟三种 Kp 下，初始偏差 1.0 的收敛过程（二阶欠阻尼示意）
  function simResponse(zeta, wn, n) {
    var data = [];
    for (var t = 0; t <= n; t++) {
      var x = t * 0.05;
      var wd = wn * Math.sqrt(1 - zeta * zeta);
      var phi = Math.acos(zeta);
      var y = Math.exp(-zeta * wn * x) * Math.cos(wd * x - phi) / Math.sqrt(1 - zeta * zeta);
      data.push([x, Math.abs(y) < 0.002 ? 0 : +y.toFixed(3)]);
    }
    return data;
  }
  var slowData = [], t;
  for (t = 0; t <= 120; t++) {                    // Kp 太小：无超调但极慢
    var x = t * 0.05;
    slowData.push([x, +Math.exp(-0.35 * x).toFixed(3)]);
  }

  var chart = echarts.init(document.getElementById('chart-pid'), null, { renderer: 'svg' });
  chart.setOption({
    animation: false,
    color: [muted, accent, accent2],
    tooltip: { trigger: 'axis', appendToBody: true },
    legend: {
      data: ['Kp 太小：缓慢爬行', 'Kp 合适：快速收敛', 'Kp 太大：持续振荡'],
      textStyle: { color: ink }, top: 0
    },
    grid: { left: 50, right: 30, top: 40, bottom: 40 },
    xAxis: {
      type: 'value', name: '时间 (s)', nameTextStyle: { color: muted },
      axisLabel: { color: muted }, splitLine: { lineStyle: { color: rule } }
    },
    yAxis: {
      type: 'value', name: '偏差', min: -1.2, max: 1.2,
      nameTextStyle: { color: muted },
      axisLabel: { color: muted }, splitLine: { lineStyle: { color: rule } }
    },
    series: [
      {
        name: 'Kp 太小：缓慢爬行', type: 'line', showSymbol: false,
        data: slowData, lineStyle: { width: 2.5, type: 'dashed' }
      },
      {
        name: 'Kp 合适：快速收敛', type: 'line', showSymbol: false,
        data: simResponse(0.5, 2.2, 120), lineStyle: { width: 3 }
      },
      {
        name: 'Kp 太大：持续振荡', type: 'line', showSymbol: false,
        data: simResponse(0.12, 2.2, 120), lineStyle: { width: 2.5 }
      }
    ]
  });
  window.addEventListener('resize', function() { chart.resize(); });
})();
