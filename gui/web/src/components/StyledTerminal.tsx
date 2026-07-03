import styled from "styled-components";

export const StyledTerminal = styled.div`
  background: var(--bg-card-solid);
  border-radius: 12px;
  overflow: hidden;

  div.react-terminal-wrapper {
    padding-top: 10px;
    height: 100%;
    background: var(--bg-card-solid) !important;
  }

  div.react-terminal-wrapper > div.react-terminal-window-buttons {
    display: none;
  }

  div.react-terminal {
    font-size: 12px;
    line-height: 1.5;
  }
`;
