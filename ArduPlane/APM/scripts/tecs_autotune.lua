-- SD:/APM/scripts/tecs_autotune.lua
-- CH7 HIGH -> +30 m ALT step, tepkiye göre TECS paramlarını küçük adımlarla ayarla.

local STEP_DH        = 30       -- [m] irtifa step (STE testi)
local BAND_PCT       = 0.02     -- ±%2 yerleşme bandı
local TIMEOUT        = 20       -- [s] güvenlik zaman aşımı
local TARGET_SETTLE  = 8.0      -- [s] hedef yerleşme
local TARGET_OVERSHOOT = 10.0   -- [%] hedef aşma

local DT             = 0.2      -- [s] döngü periyodu
local PHASE          = 0        -- 0: bekle, 1: step atıldı, ölç
local t0             = 0
local h0, href       = 0, 0
local y_peak, y_trough = -1e9, 1e9
local iae, itae      = 0, 0
local t_rel          = 0

local function time_s()
  return millis():tofloat()/1000.0
end

local function ch7_high()
  local pwm = rc:get_pwm(7)
  return pwm and pwm > 1800
end

local function get_alt()
  return ahrs:get_relative_altitude()  -- [m]
end

local function set_alt_ref(h)
  -- En basit yol: mevcut hedefe offset veriyoruz (AUTO/LOITER hedefini değiştirir)
  -- Pratikte görev/waypoint hedefi kullanmak daha temizdir; POC için bu yeterli.
  param:set("ALT_HOLD_RTL", math.max(10, h))  -- acil durumlarda yüksekliği güvenli tut
  -- Not: Basitçe referansı doğrudan değiştirmek yerine görev komutu ile vermek tercih edilir.
end

-- Kısa yol param yardımcıları
local function pget(name) return param:get(name) end
local function pset(name, v) param:set(name, v) ; return param:get(name) end

local function reset_measure()
  t0    = time_s()
  h0    = get_alt()
  href  = h0 + STEP_DH
  y_peak, y_trough = -1e9, 1e9
  iae, itae = 0, 0
  t_rel = 0
end

local function apply_step()
  gcs:send_text(6, string.format("TECS-AT: ALT step +%dm", STEP_DH))
  -- referans irtifayı artır (basit POC)
  -- Gerçekte: DO_CHANGE_ALT komutuyla vermek daha temiz.
  href = h0 + STEP_DH
  -- set_alt_ref(href) -- görevle veriyorsan bu satırı kendine göre uygula
end

local function update_measure()
  local t = time_s()
  t_rel = t - t0

  local y = get_alt()
  if y > y_peak then y_peak = y end
  if y < y_trough then y_trough = y end

  local e = (href - y)
  iae  = iae  + math.abs(e)*DT
  itae = itae + math.abs(e)*t_rel*DT

  local step_size = href - h0
  local band = math.max(math.abs(step_size)*BAND_PCT, 1.0) -- en az 1 m

  if (math.abs(y - href) < band and t_rel > 2.0) or (t_rel > TIMEOUT) then
    -- Karar ver ve paramları ufak ufak ayarla
    local settle = t_rel
    local overs  = 0.0
    if step_size > 0 then overs = math.max(0.0, (y_peak - href)/math.abs(step_size)*100.0)
    elseif step_size < 0 then overs = math.max(0.0, (href - y_trough)/math.abs(step_size)*100.0) end

    gcs:send_text(6, string.format(
      "TECS-AT: settle=%.1fs overs=%.1f%% IAE=%.2f ITAE=%.2f",
      settle, overs, iae, itae))

    local changed = false
    -- 1) Geç oturuyor -> TIME_CONST azalt
    if settle > TARGET_SETTLE then
      local v = pget("TECS_TIME_CONST") or 5.0
      v = math.max(1.0, v - 0.2)
      v = pset("TECS_TIME_CONST", v)
      gcs:send_text(6, string.format("TECS_TIME_CONST -> %.2f", v))
      changed = true
    end
    -- 2) Overshoot yüksek -> sönüm artır
    if overs > TARGET_OVERSHOOT then
      local pd = pget("TECS_PTCH_DAMP") or 0.5
      local td = pget("TECS_THR_DAMP")  or 0.5
      pd = pset("TECS_PTCH_DAMP", pd + 0.05)
      td = pset("TECS_THR_DAMP",  td + 0.05)
      gcs:send_text(6, string.format("PTCH_DAMP=%.2f THR_DAMP=%.2f", pd, td))
      changed = true
    end
    -- 3) Kalıcı ofset varsa integratör biraz artır
    if math.abs(href - y) > 0.5 then
      local ig = pget("TECS_INTEG_GAIN") or 0.02
      ig = pset("TECS_INTEG_GAIN", ig + 0.005)
      gcs:send_text(6, string.format("INTEG_GAIN=%.3f", ig))
      changed = true
    end

    if not changed then
      gcs:send_text(6, "TECS-AT: hedeflere yakin, degisim yapilmadi")
    end
    PHASE = 0
  end
end

function update()
  -- yalnızca LOITER / AUTO gibi modlarda çalıştır (örnek kontrol)
  local mode = vehicle:get_mode()
  -- LOITER=5, AUTO=10; kart/sürüm farkı olabilir
  if mode ~= 5 and mode ~= 10 then
    return update, 1000
  end

  if PHASE == 0 then
    if ch7_high() then
      reset_measure()
      apply_step()
      PHASE = 1
    end
  elseif PHASE == 1 then
    update_measure()
  end

  return update, math.floor(DT*1000)
end

return update()
