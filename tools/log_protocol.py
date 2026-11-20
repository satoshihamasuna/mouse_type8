"""保存ログのHEADERからバイナリフレーム長を決定する。"""
import struct

IR_TRAILER = ('sen_l.value', 'sen_r.value', 'sen_fl.value', 'sen_fr.value', 'ir.range_states', 'motion.pattern')


def saved_log_frame(labels):
    if not labels or labels[0] != 'cnt':
        raise ValueError('ログのHEADERが必要です。log mode0に設定して再取得してください。')
    count = len(labels) - 1
    if count not in (51, 55):
        raise ValueError('未対応のログ項目数: {}'.format(count))
    return struct.Struct('<HH' + 'H' * count)


def saved_log_magic(labels):
    saved_log_frame(labels)  # Validate the HEADER before assigning a schema.
    if len(labels) == 56 and tuple(labels[-6:]) == IR_TRAILER:
        if (tuple(labels[1:5]) == ('ideal.velo', 'ego.velo', 'ideal.accel', 'ego.accel')
                and tuple(labels[36:40]) == ('irq.total_us', 'irq.logging_us', 'irq.max_us', 'irq.period_us')):
            if labels[13] == 'estimate.heading_rad' and labels[15] == 'estimate.sigma_heading_rad':
                return 0xA55D
            return 0xA55C
        return 0xA55B
    return 0xA55A


def decode_saved_values(labels, words):
    """First 49 words are binary16 in A55B/C/D; preserve the integer trailer exactly."""
    mixed = saved_log_magic(labels) in (0xA55B, 0xA55C, 0xA55D)
    if len(words) != len(labels)-1:
        raise ValueError('ログのHEADERとデータ長が一致しません')
    half_count = 49 if mixed else len(words)
    values = [struct.unpack('<e', struct.pack('<H', w))[0] for w in words[:half_count]]
    if mixed:
        values.extend(w if w < 0x8000 else w - 0x10000 for w in words[49:53])
        values.extend(words[53:])
    return values
