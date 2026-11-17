from robot_control.crsf_protocol import (
    CRSF_CHANNEL_CENTER,
    CRSF_RC_CHANNEL_COUNT,
    CRSF_RC_CHANNELS_PACKED,
    CRSF_RC_FRAME_LENGTH,
    crsf_crc8,
    CrsfFrameParser,
)


def make_rc_frame(channels: list[int]) -> bytes:
    packed_channels = sum(
        value << (11 * index)
        for index, value in enumerate(channels)
    )
    payload = packed_channels.to_bytes(22, byteorder='little')
    frame_type_and_payload = bytes([CRSF_RC_CHANNELS_PACKED]) + payload
    checksum = crsf_crc8(frame_type_and_payload)
    return (
        bytes([0xC8, CRSF_RC_FRAME_LENGTH])
        + frame_type_and_payload
        + bytes([checksum])
    )


def test_crc8_matches_crsf_dvb_s2_check_value() -> None:
    assert crsf_crc8(b'123456789') == 0xBC


def test_parser_handles_partial_frames_and_normalizes_channels() -> None:
    raw_channels = [CRSF_CHANNEL_CENTER] * CRSF_RC_CHANNEL_COUNT
    raw_channels[0] = 172
    raw_channels[3] = 1811
    frame = make_rc_frame(raw_channels)
    parser = CrsfFrameParser()

    assert parser.feed(frame[:7]) == []
    decoded = parser.feed(frame[7:])

    assert len(decoded) == 1
    assert decoded[0][0] == -1.0
    assert decoded[0][3] == 1.0
    assert decoded[0][4] == 0.0
    assert len(decoded[0]) == CRSF_RC_CHANNEL_COUNT


def test_parser_discards_bad_crc_and_recovers_next_frame() -> None:
    raw_channels = [CRSF_CHANNEL_CENTER] * CRSF_RC_CHANNEL_COUNT
    invalid_frame = bytearray(make_rc_frame(raw_channels))
    invalid_frame[-1] ^= 0xFF
    parser = CrsfFrameParser()

    decoded = parser.feed(bytes(invalid_frame) + make_rc_frame(raw_channels))

    assert len(decoded) == 1
    assert all(value == 0.0 for value in decoded[0])


def test_parser_ignores_non_rc_frames() -> None:
    frame_type_and_payload = bytes([0x14, 0x00])
    checksum = crsf_crc8(frame_type_and_payload)
    frame = bytes([0xC8, 3]) + frame_type_and_payload + bytes([checksum])

    assert CrsfFrameParser().feed(frame) == []
