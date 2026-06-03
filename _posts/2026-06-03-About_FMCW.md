---
title: About FMCW (Part 1) — From Chirp to Range-Doppler
date: 2026-06-03 05:00:00 +0900
categories: [Radar, Signal Processing]
tags: [fmcw, radar, rf, dsp, chirp, beat-frequency]
media_subpath: /assets/img/2026-06-03/
image: range_fft.png
toc: true
comments: true
pin: true
math: true
mermaid: false
---
<hr>
<br>

> A from-scratch derivation of **FMCW (Frequency-Modulated Continuous Wave) radar** —
> how transmitting a linearly-swept *chirp* and mixing it with the echo turns a tiny
> **beat frequency** into a target's **range**.

<br>
<hr>

## The Basic Idea

FMCW radar continuously transmits a **chirp**: a signal whose frequency ramps up linearly in time.
The echo from a target comes back **delayed**, so at any instant the received frequency differs
from the transmitted one by a small amount. That difference — the **beat frequency** — is directly
proportional to distance.

Two definitions everything is built on:

| Symbol | Meaning |
|--------|---------|
| $T = \dfrac{1}{f}$ | period = 1 / frequency |
| $B = f_f - f_i$ | **bandwidth** = (end frequency − start frequency) of the sweep |

<br>
<hr>

## The Transmitted Signal (Chirp)

The transmitted voltage is a cosine whose phase grows quadratically (because frequency grows linearly):

$$V_T(t) = \cos\!\big[\, w_c t + \pi B f_r t^2 \,\big] = \cos\big[\phi_T(t)\big]$$

| Symbol | Meaning |
|--------|---------|
| $w_c = 2\pi f_c$ | carrier angular frequency |
| $f_c$ | **carrier frequency** (e.g. 10 MHz) |
| $B$ | sweep bandwidth |
| $f_r = 1/T_r$ | **pulse repetition frequency (PRF)** ($T_r$ = pulse repetition interval) |
| $\phi_T(t)$ | total phase of the transmitted signal |

> One cycle of $2\pi$ per second = 1 Hz. With $f_c = 10\,\text{MHz}$, the carrier completes 10 million cycles per second.

<br>
<hr>

## Instantaneous Frequency

The instantaneous frequency is the phase derivative divided by $2\pi$:

$$\phi_T(t) = w_c t + \pi B f_r t^2$$

$$f_T(t) = \frac{1}{2\pi}\frac{d\phi_T(t)}{dt} = \frac{w_c + 2\pi B f_r t}{2\pi} \;\Longrightarrow\; \boxed{\,f_T(t) = f_c + B f_r\, t\,}$$

So the frequency rises **linearly** with slope $B f_r$. Over one sweep the total swing is exactly the bandwidth:

$$\Delta f_T = B = 100\,\text{kHz}\qquad(\text{e.g. } 10\,\text{MHz} \to 10.1\,\text{MHz},\;\; T_r = 1\,\text{s})$$

*(A chirp looks like a sinusoid whose oscillations get tighter and tighter across the sweep — frequency climbing from the start to the end of the band.)*

<br>
<hr>

## The Echo: Round-Trip Delay

If the target sits at range $R$ and moves with velocity $V$:

$$R(t) = R_0 + V t$$

The signal has to travel **there and back**, so the echo is delayed by the round-trip time:

$$t_d = \frac{2R(t)}{c}$$

> The factor **2** is the round trip. $c$ is the speed of light (RF).

After transmission the echo arrives a little later — this is the **Tx → Rx** delay we exploit.

<br>
<hr>

## The Received Signal

The received signal is just a **delayed, attenuated copy** of the transmitted one:

$$V_R(t) = A\, V_T(t - t_d) = A\cos\!\big[\, w_c(t - t_d) + \pi B f_r (t - t_d)^2 \,\big] = A\cos\big[\phi_T(t - t_d)\big]$$

<br>
<hr>

## Mixing (Tx × Rx)

To pull out the frequency difference, we **multiply** the transmitted and received signals (the *mixer*),
then use the product-to-sum identity:

$$2\cos\alpha\cos\beta = \cos(\alpha - \beta) + \cos(\alpha + \beta)$$

Multiplying $V_T \cdot V_R$ gives two terms:

$$
\underbrace{A\cos\!\big[\, w_c(t-t_d) - w_c t + \pi B f_r (t-t_d)^2 - \pi B f_r t^2 \,\big]}_{\textbf{difference} \;\rightarrow\; \text{low frequency (the beat)}}
\;+\;
\underbrace{A\cos\!\big[\, w_c t + w_c(t-t_d) + \pi B f_r t^2 + \pi B f_r (t-t_d)^2 \,\big]}_{\textbf{sum} \;\rightarrow\; \text{very high frequency}}
$$

A **low-pass filter** then throws away the sum (high-frequency) term and keeps only the difference (beat) term.

<br>
<hr>

## Beat Frequency → Range

Remember the **difference term** we kept after the low-pass filter — its phase is $\alpha - \beta$,
i.e. $\phi_R - \phi_T$, which from the mixing step was

$$\phi_R - \phi_T = w_c(t-t_d) - w_c t + \pi B f_r (t-t_d)^2 - \pi B f_r t^2$$

Expand it step by step — the $w_c t$ and the $t^2$ pieces partly cancel:

$$
\begin{aligned}
\phi_R - \phi_T &= -w_c t_d + \pi B f_r\big[(t-t_d)^2 - t^2\big] \\
&= -w_c t_d + \pi B f_r\big[-2t\,t_d + t_d^2\big] \\
&= \underbrace{-w_c t_d + \pi B f_r t_d^2}_{\text{constant phase}}\;\underbrace{-\,2\pi B f_r t_d\,t}_{\text{grows with } t}
\end{aligned}
$$

Only the last term changes with time, so **it** is what sets the frequency. The **beat frequency** is that
slope divided by $2\pi$:

$$f_{\text{beat}} = \frac{1}{2\pi}\left|\frac{d(\phi_R - \phi_T)}{dt}\right| = \frac{1}{2\pi}\cdot 2\pi B f_r t_d = B f_r\, t_d = B f_r \cdot \frac{2R}{c}$$

Solving for range gives the punchline of FMCW radar:

$$\boxed{\, R = \frac{c\, f_{\text{beat}}}{2 B f_r} \,}$$

In other words: **transmit a chirp, mix it with the echo, low-pass filter, and measure the leftover beat
frequency — that single number tells you the distance to the target.**

<br>
<hr>

## Letting the Target Move: the Beat Phase

So far the target was effectively still. Now let it move with velocity $V$ and track the **beat phase**
over the *fast time* $t_i$ (time within a single chirp). After mixing, the beat phase is the phase difference:

$$\phi_I(t_i) = \phi_T(t_i - t_d) - \phi_T(t_i)$$

With a moving target the delay itself depends on time:

$$t_d = \frac{2R(t_i)}{c} = \frac{2(R_0 + V t_i)}{c} = t_0 + \frac{2V}{c}t_i,\qquad t_0 \equiv \frac{2R_0}{c}$$

Expanding with $\phi_T(t) = w_c t + \pi B f_r t^2$:

$$\phi_I(t_i) = -w_c t_d - 2\pi B f_r\, t_i t_d + \pi B f_r t_d^2$$

Substitute $t_d$ and **drop the $t_i^2$ terms** (negligible since $V \ll c$), using $w_c = 2\pi f_c$:

$$\boxed{\;\phi_I(t_i) \simeq \phi_0 - 2\pi\!\left[\frac{2V}{c}f_c + B f_r t_0\right]t_i\;}$$

where $\phi_0$ gathers all the constant (time-independent) phase.

<br>
<hr>

## Range and Doppler in the Beat Frequency

The coefficient of $t_i$ above **is** the beat frequency:

$$\boxed{\;f_{\text{beat}} = \underbrace{\frac{2V}{c}f_c}_{\text{Doppler }f_d} + \underbrace{B f_r\, t_0}_{\text{range beat}}\;}$$

Now two physical effects are mixed into one beat:

| Term | Source | Meaning |
|------|--------|---------|
| $\dfrac{2V}{c}f_c$ | target **velocity** | Doppler shift $f_d$ |
| $B f_r\, t_0 = \dfrac{2 B f_r R_0}{c}$ | target **range** | the range beat from before |

So in principle a *single* chirp's beat mixes range and velocity. But notice that the **range beat is the
chirp slope $S = B f_r$ times the delay**, $\;B f_r t_0 = S\,t_0\,$, whereas the Doppler term $\frac{2V}{c}f_c$
does **not** depend on the slope at all (only on carrier and velocity).

Sweep a **large bandwidth $B$ in a very short time** — a *fast chirp*, big slope $S$ — and the range term
$S\,t_0$ blows up while the Doppler stays fixed. The velocity contribution then becomes **negligible within
one chirp**, and the beat is effectively **pure range**:

$$f_{\text{beat}} \approx B f_r\, t_0 \qquad\big(\text{fast chirp: } S\,t_0 \gg \tfrac{2V}{c}f_c\big)$$

> **One chirp → range. Many chirps → velocity.**
> A single chirp gives *one* measurement (the beat frequency) but *two* unknowns ($R$ and $V$) — it's
> underdetermined, so you can only read it as **range** (and a fast chirp makes the velocity term negligible
> anyway). Velocity is **not** recoverable from one beat number; it only appears in the **phase change of the
> range peak across many chirps** (slow time). That is exactly what the next section does.
{: .prompt-tip }

<br>
<hr>

## Many Chirps: Pulse Index and Range-Doppler

Transmit a train of identical chirps and label them with a **pulse index** $n$ (this is the *slow time*).
Within the $n$-th chirp the absolute time is

$$t = n T_r + t_i$$

so the range — and therefore the delay — also accumulates the chirp-to-chirp motion:

$$t_d = \frac{2R}{c} = \frac{2\big(R_0 + V(nT_r + t_i)\big)}{c} = t_0 + \frac{2V}{c}nT_r + \frac{2V}{c}t_i$$

Repeating the beat-phase expansion, the same general form
$\phi_I = -w_c t_d - 2\pi B f_r\, t_i t_d + \pi B f_r t_d^2$ now carries the $n$-dependent delay
$t_d = t_0 + \frac{2V}{c}nT_r + \frac{2V}{c}t_i$. Writing every term out:

$$
\begin{aligned}
\phi_{I,n}(t_i) &= \phi_T(t_i - t_d) - \phi_T(t_i) \\
&= -w_c t_0 - w_c\tfrac{2V}{c}nT_r - w_c\tfrac{2V}{c}t_i
   - 2\pi B f_r\!\left[t_0 + \tfrac{2V}{c}nT_r + \tfrac{2V}{c}t_i\right]t_i
   + \pi B f_r\!\left[t_0 + \tfrac{2V}{c}nT_r + \tfrac{2V}{c}t_i\right]^2
\end{aligned}
$$

Only the **two $t_i^2$ terms** — one from $\frac{2V}{c}t_i\cdot t_i$, one from $\left(\frac{2V}{c}t_i\right)^2$ —
are dropped. **Every term linear in $t_i$ is kept**, including the $n$-dependent one
$B f_r\frac{2V}{c}nT_r = \frac{2VBn}{c}$ (using $f_r T_r = 1$). Collecting the constants into $\phi_0$ and
using $w_c = 2\pi f_c$:

$$\phi_{I,n}(t_i) \simeq \phi_0 \;-\; \underbrace{2\pi f_c\frac{2V}{c}\,n T_r}_{\text{slow time (Doppler)}} \;-\; \underbrace{2\pi\!\left[\frac{2V}{c}f_c + B f_r t_0 + \frac{2VBn}{c}\right]t_i}_{\text{fast-time beat, incl. range walk}}$$

This is the heart of FMCW range-Doppler processing:

- **Fast time $t_i$** → the beat frequency carries **range** → an FFT *within* each chirp.
- **Slow time $nT_r$** → the phase advances linearly with $n$ at the Doppler rate $f_c\frac{2V}{c}$ → an FFT *across* chirps recovers **velocity**.
- The extra $\frac{2VBn}{c}$ inside the beat is **range walk (range migration)**: it maps to a range shift $\delta R = VnT_r$ — exactly how far the target moved by chirp $n$. Tiny per chirp, but it smears fast targets across range bins over a long frame.

Two FFTs — one within, one across chirps — separate $R_0$ and $V$ into a 2-D **range-Doppler map**.
The next section makes that *first* FFT explicit.

<br>
<hr>

## Double-FFT Digital Processing (the Range FFT)

In hardware the beat signal is sampled and **Fourier-transformed**. Recall the per-chirp beat frequency:

$$f_{I,n} = \frac{2V}{c}f_c + B f_r t_0 + \frac{2VBn}{c}$$

Take the **first FFT** over the fast time $t_i \in \left[-\tfrac{T}{2},\, \tfrac{T}{2}\right]$ (one chirp):

$$V_{I,n}(f) = \int_{-T/2}^{T/2} A\cos\big[\phi_{I,n}(t_i)\big]\, e^{-2\pi i f t_i}\, dt_i$$

Writing the cosine as two exponentials and integrating turns each into a **sinc**:

$$V_{I,n}(f) = \frac{AT}{2}\left\{ \frac{\sin\!\big[2\pi(f - f_{I,n})\tfrac{T}{2}\big]}{2\pi(f - f_{I,n})\tfrac{T}{2}}\, e^{-i\phi_0 + i2\pi f_c\frac{2V}{c}nT_r} \;+\; \frac{\sin\!\big[2\pi(f + f_{I,n})\tfrac{T}{2}\big]}{2\pi(f + f_{I,n})\tfrac{T}{2}}\, e^{+i\phi_0 - i2\pi f_c\frac{2V}{c}nT_r} \right\}$$

![First FFT of the beat signal: a sinc peaked at the beat frequency](range_fft.png)
_First FFT (range FFT): one chirp's beat signal transforms into a sinc, peaked at the beat frequency — the peak location gives range._

Two things to read off:

- The first sinc **peaks at $f = f_{I,n}$** (the second at $f = -f_{I,n}$). That peak location is the beat frequency → **range**. This is the *range FFT*.
- The peak's phase $e^{-i\phi_0 + i2\pi f_c\frac{2V}{c}nT_r}$ advances **linearly with the chirp index $n$** at the Doppler rate $f_c\frac{2V}{c}$. A **second FFT across $n$** (slow time) reads that phase progression → **velocity**.

That is **double-FFT processing**: FFT #1 over fast time gives range, FFT #2 over slow time (chirp index) gives velocity — together the 2-D **range-Doppler map**.

Concretely the samples form a matrix — one **row per chirp** (slow time $n$), one **column per fast-time
sample** — and the 2-D FFT turns it into the range-Doppler map, where the target is a single bright cell:

$$
\begin{bmatrix}
s_{0,0} & s_{0,1} & \cdots & s_{0,M-1}\\
s_{1,0} & s_{1,1} & \cdots & s_{1,M-1}\\
\vdots & \vdots & \ddots & \vdots\\
s_{N-1,0} & s_{N-1,1} & \cdots & s_{N-1,M-1}
\end{bmatrix}
\;\xrightarrow{\;\text{2-D FFT}\;}\;
\begin{bmatrix}
0 & 0 & 0 & 0 & 0\\
0 & 0 & 0 & 0 & 0\\
0 & 0 & 1 & 0 & 0\\
0 & 0 & 0 & 0 & 0
\end{bmatrix}
$$

Rows = chirp index $n$ (→ **Doppler / velocity** after the FFT), columns = fast-time samples
(→ **range** after the FFT). The lone **1** marks the target's (range, velocity) cell.

![2-D FFT matrix calculation: raw data matrix to range-Doppler result](double_fft_matrix.png)
_The matrix calculation: FFT along each row (fast time) gives range; FFT along each column (across chirps) gives velocity; the target ends up in one cell._

**The matrix dimensions make the "one chirp = range" point exact.** The raw data is $N \times M$ ($N$ chirps,
$M$ fast-time samples each). After the **1st FFT** (per chirp, over its $M$ samples) the beat — a *real* signal —
keeps only its $M/2$ positive-frequency bins, so each chirp becomes an **$M/2 \times 1$ column of range bins**:
one chirp carries **range only**. Stack all $N$ chirps side by side → an **$M/2 \times N$ matrix** (range × chirp).
A **2nd FFT along the chirp axis $N$** turns each range bin's chirp-to-chirp values into a **velocity spectrum**
(amplitude vs velocity) — so velocity appears only once you have the $N$ chirps. The final range-Doppler matrix
is $M/2 \times N$: range bins × velocity bins, with the target at one cell.

<br>
<hr>

## Summary

1. Transmit a linear chirp: $f_T(t) = f_c + B f_r t$.
2. The echo returns delayed by $t_d = 2R/c$.
3. Mixing Tx × Rx produces a **sum** term and a **difference** term.
4. Low-pass filtering keeps the **beat** (difference) term.
5. For a still target the beat is proportional to range: $R = \dfrac{c\, f_{\text{beat}}}{2 B f_r}$.
6. For a moving target the beat also picks up Doppler: $f_{\text{beat}} = \dfrac{2V}{c}f_c + B f_r t_0$.
7. Across many chirps (pulse index $n$) the phase splits into a fast-time **beat (range)** and a slow-time **Doppler (velocity)** progression — two FFTs give a **range-Doppler map**.
8. **Double-FFT**: FFT #1 over fast time gives a sinc peaked at $f_{I,n}$ → **range**; FFT #2 over slow time (chirp index $n$) reads the Doppler phase → **velocity**.
