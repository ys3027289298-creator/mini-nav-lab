# corner

拐角折线与动画渐进转向曲线不匹配时，用过渡点把单段转角压到滑步预算内。

```bash
make test
./corner --csv   # 输出 P=0..6 的滑步/长度曲线
```

核心式子（角度用弧度）：

\[
d_{\rm slip}=\frac12\theta vT,\qquad
\theta_{\max}=\frac{2b}{vT}=\frac{\ell_{\rm foot}}{vT}.
\]

若转向档给的是角速度 \(\omega\)，则 \(T=\theta/\omega\)，从而

\[
d_{\rm slip}=\frac{v\theta^2}{2\omega},\qquad
\theta_{\max}=\sqrt{\frac{\omega\ell_{\rm foot}}{v}}.
\]

自检使用可替换的演示尺度：脚掌 \(0.27\rm\,m\)、半脚掌 \(b=0.135\rm\,m\)、
速度 \(1.4\rm\,m/s\)、直角弯、原路径长 \(8\rm\,m\)、每个过渡点的实测外绕
增量 \(0.10\rm\,m\)。三档动画转向窗口 \(0.45/0.30/0.18\rm\,s\)（等效直角转向
速度 \(200/300/500^\circ/s\)）分别需要 \(3/2/1\) 个过渡点；峰值滑步均小于
半脚掌，路径增幅分别为 \(3.75\%/2.50\%/1.25\%\)，不超过 \(5\%\)。
