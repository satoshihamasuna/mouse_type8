"""保存ログのHEADERからバイナリフレーム長を決定する。"""
import struct


def saved_log_frame(labels):
    if not labels or labels[0] != 'cnt':
        raise ValueError('ログのHEADERが必要です。log mode0に設定して再取得してください。')
    count = len(labels) - 1
    if count not in (51, 55):
        raise ValueError('未対応のログ項目数: {}'.format(count))
    return struct.Struct('<HH' + 'H' * count)
