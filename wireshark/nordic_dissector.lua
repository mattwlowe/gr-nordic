-- trivial protocol example
-- declare our protocol
nordic_proto = Proto("nordic","Nordic Semiconductor nRF24L")

nordic_proto.fields.address = ProtoField.bytes("nordic.address", "Address", base.SPACE)
nordic_proto.fields.address_length = ProtoField.uint8("nordic.address_length", "Address length", base.DEC)
nordic_proto.fields.channel = ProtoField.uint8("nordic.channel", "Channel", base.DEC)
nordic_proto.fields.data_rate = ProtoField.string("nordic.data_rate", "Data rate")
nordic_proto.fields.payload = ProtoField.bytes("nordic.payload", "Payload", base.SPACE)
nordic_proto.fields.payload_length = ProtoField.uint8("nordic.payload_length", "Payload length", base.DEC)
nordic_proto.fields.crc = ProtoField.uint16("nordic.crc", "CRC", base.HEX)
nordic_proto.fields.crc_length = ProtoField.uint8("nordic.crc_length", "CRC length", base.DEC)
nordic_proto.fields.no_ack = ProtoField.bool("nordic.no_ack", "No ACK")
nordic_proto.fields.ack = ProtoField.bool("nordic.ack", "ACK")
nordic_proto.fields.ack_retransmits = ProtoField.uint8("nordic.ack_retransmits", "ACK retransmits")
nordic_proto.fields.sequence_number = ProtoField.uint8("nordic.sequence_number","Sequence number", base.DEC)
nordic_proto.fields.big_packet = ProtoField.bool("nordic.big_packet","Big packet (DPL)")

-- create a function to dissect it
function nordic_proto.dissector(buffer,pinfo,tree)
    pinfo.cols.protocol = nordic_proto.name
    local subtree = tree:add(nordic_proto,buffer(),"Nordic Enhanced ShockBurst (ESB)")
	local hd_subtree = subtree:add("GNU Radio Tags")
    pinfo.cols.info = ""

    -- channel
    local channel = buffer(0,1):uint()

    -- data rate
    local data_rate = buffer(1,1):uint()
    local data_rate_string = ""
    if data_rate == 0 then data_rate_string = "250 kbps" end
    if data_rate == 1 then data_rate_string = "1 mbps" end
    if data_rate == 2 then data_rate_string = "2 mbps" end

    -- address length
    local address_length = buffer(2,1):uint()

    -- payload length
    local payload_length = buffer(3,1):uint()

    -- sequence number
    local sequence_number = buffer(4,1):uint()

    -- no ack bit
    local no_ack = buffer(5,1):uint()

    -- crc length
    local crc_length = buffer(6,1):uint()

    -- dynamic payloads bit
    local big_packet = buffer(7,1):uint()

    -- address
    local address = buffer(8,address_length)
    local address_bytes = address:bytes()

    -- payload
    local payload = buffer(8+address_length,payload_length)
    local payload_bytes = payload:bytes()
	local ack = False
	
	-- TODO - ack packet is not well understood or documented. Length 0 is one type of ack packet, but some can have payloads as well
	-- ack_retransmits - not yet implemented
	local ack_retransmits = 0
	
    -- TODO 
    if payload_bytes:len() == 0 then
      pinfo.cols.info = "ACK"
	  ack = True
    end
	
	-- TODO
	-- bad_crc = buffer(8,1):uint()
	bad_crc = 0
	

    -- crc
    local crc = buffer(8+address_length+payload_length, crc_length)
	hd_subtree:add(nordic_proto.fields.channel, buffer(0,1), channel)
	hd_subtree:add(nordic_proto.fields.data_rate, buffer(1,1), data_rate_string)
	hd_subtree:add(nordic_proto.fields.address_length, buffer(2,1), address_length)
	hd_subtree:add(nordic_proto.fields.crc_length, buffer(6,1), crc_length)

	subtree:add(nordic_proto.fields.address, address)
	subtree:add(nordic_proto.fields.sequence_number, buffer(4,1), sequence_number)
	subtree:add(nordic_proto.fields.no_ack, buffer(5,1), no_ack)
	subtree:add(nordic_proto.fields.big_packet, buffer(7,1), big_packet)
	subtree:add(nordic_proto.fields.ack, ack)
	subtree:add(nordic_proto.fields.ack_retransmits, ack_retransmits)
	subtree:add(nordic_proto.fields.payload_length, buffer(3,1), payload_length)
	subtree:add(nordic_proto.fields.payload, payload)
	crcNode = subtree:add(nordic_proto.fields.crc, crc)
	
	if (bad_crd == 1) then
		crcNode:add_expert_info(PI_CHECKSUM, PI_ERROR, "CRC incorrect")
	end

    -- Microsoft
    if bit.band(buffer(7,1):uint(), 0xF0) == 0xA0 and payload_length > 0 then 

      -- Validate the checksum (AES encrypted series)
      local sum = 0xFF
      for x = 0, payload_bytes:len()-1 do
        sum = bit.bxor(sum, payload_bytes:get_index(x))
      end
      sum = bit.band(sum, 0xFF)
      if sum == 0 then

        -- Microsoft keepalive
        if payload_bytes:get_index(1) == 0x38 and payload_bytes:len() == 8 then
          pinfo.cols.info = "Keepalive"
        end

        -- Microsoft mouse movement/click 
        if payload_bytes:get_index(1) == 0x90 then

          local vtree = subtree:add(nordic_proto, buffer(), "Microsoft Movement/Click")
          pinfo.cols.info = "Microsoft Mouse Movement/Click"
          vtree:add(string.format("Device Type: 0x%02x", payload_bytes:get_index(2)))

          -- Movement X/Y
          local movement_x = payload(9, 2):le_int()
          local movement_y = payload(11, 2):le_int()
          local scroll = payload(13, 2):le_int()
          vtree:add(string.format("Movement X:   %i", movement_x))
          vtree:add(string.format("Movement Y:   %i", movement_y))
          vtree:add(string.format("Scroll Wheel: %i", scroll))

          -- Button mask 
          local button_mask = payload(8, 1):uint()
          vtree:add(string.format("Button 0:     %i", bit.band(bit.rshift(button_mask, 0), 1)))
          vtree:add(string.format("Button 1:     %i", bit.band(bit.rshift(button_mask, 1), 1)))
          vtree:add(string.format("Button 2:     %i", bit.band(bit.rshift(button_mask, 2), 1)))
          vtree:add(string.format("Button 3:     %i", bit.band(bit.rshift(button_mask, 3), 1)))
          vtree:add(string.format("Button 4:     %i", bit.band(bit.rshift(button_mask, 4), 1)))
          vtree:add(string.format("Button 5:     %i", bit.band(bit.rshift(button_mask, 5), 1)))
          vtree:add(string.format("Button 6:     %i", bit.band(bit.rshift(button_mask, 6), 1)))
          vtree:add(string.format("Button 7:     %i", bit.band(bit.rshift(button_mask, 7), 1)))

        end

      end
    end

end

-- load the udp.port table
udp_table = DissectorTable.get("udp.port")

-- register our protocol to handle udp port 7777
udp_table:add(9451,nordic_proto)